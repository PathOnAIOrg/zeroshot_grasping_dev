from pathlib import Path
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to avoid Tkinter issues
from matplotlib import pyplot as plt
from flask import Flask, request, jsonify, send_from_directory
import logging
import numpy as np
import torch
import os
import cv2
import open3d as o3d
import base64
from PIL import Image
import io
import argparse
from engine import grasp_model
from langsam import langsamutils
from langsam.langsam_actor import LangSAM
# from VLP.new_vlp_actor import SegmentAnythingActor
from openai import OpenAI
from grasp_detetor import Graspnet
import utils
import datetime
import json


app = Flask(__name__)
def get_args_parser():
    parser = argparse.ArgumentParser('RefTR For Visual Grounding; FGC-GraspNet For Grasp Pose Detection',
                                     add_help=False)
    parser.add_argument('--output_dir', default='outputs/roborefit',
                        help='path where to save, empty for no saving')
    parser.add_argument('--checkpoint_grasp_path', default='logs/checkpoint_fgc.tar', help='Model checkpoint path')
    parser.add_argument('--num_point', type=int, default=12000, help='Point Number [default: 20000]')
    parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
    parser.add_argument('--collision_thresh', type=float, default=0.01,
                        help='Collision Threshold in collision detection [default: 0.01]')
    parser.add_argument('--voxel_size', type=float, default=0.01,
                        help='Voxel Size to process point clouds before collision detection [default: 0.01]')
    parser.add_argument('--output_dir_grasp', default='outputs/graspnet')
    return parser
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
# Configure OpenAI API key
api_key = os.environ.get("OPENAI_API_KEY")
if not api_key:
    logging.error("OPENAI_API_KEY environment variable not set")
    client = None
else:
    try:
        client = OpenAI(api_key=api_key)
    except Exception as e:
        logging.error(f"Failed to initialize OpenAI client: {e}")
        client = None

# Initialize models
use_gpu = torch.cuda.is_available()
langsam_actor = LangSAM(use_gpu=use_gpu)
# vlp_actor = SegmentAnythingActor(
#     vlpart_checkpoint="VLP/swinbase_part_0a0000.pth", sam_checkpoint="VLP/sam_vit_h_4b8939.pth",
#     device="cuda" if use_gpu else "cpu")

def save_cropping_box_visualization(image, cropping_box, output_folder):
    # Save the cropping box visualization to file
    output_path = f"{output_folder}/cropping_box_visualization.png"
    x1, y1, x2, y2 = cropping_box
    plt.figure()
    plt.imshow(image)
    plt.gca().add_patch(plt.Rectangle((x1, y1), x2-x1, y2-y1, edgecolor='red', facecolor='none'))
    plt.title("Cropping Box Visualization")
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()  # Close figure to free memory
    logging.info(f"Cropping box visualization saved to: {output_path}")
    return output_path


def select_action(bboxes, pos_bboxes, text, actions, evaluate=True):
    distances = torch.sqrt((actions[0, :, 0] - pos_bboxes[0, 0, 0]) ** 2 +
                           (actions[0, :, 1] - pos_bboxes[0, 0, 1]) ** 2)

    num_distances_to_select = min(10, distances.numel())
    top_dist_indices = torch.topk(distances, k=num_distances_to_select, largest=False).indices
    min_dist_index = top_dist_indices[0]

    return min_dist_index.item()

# Helper functions
def load_image_as_base64(image_path):
    with open(image_path, 'rb') as image_file:
        base64_image = base64.b64encode(image_file.read()).decode('utf-8')
    return base64_image

def process_grasping_result(output):
    lines = output.strip().split('\n')
    result = {
        "selected_object": None,
        "cropping_box": None,
        "objects": [],
        "is_part": False
    }

    for i, line in enumerate(lines):
        if line.startswith("Selected Object/Object Part:"):
            selected_object = line.split(": ")[1].strip()
            if selected_object.startswith("[object part:"):
                result["is_part"] = True
                selected_object = selected_object[len("[object part:"):].strip(" ]")
            else:
                result["is_part"] = False
                selected_object = selected_object[len("[object:"):].strip(" ]")
            result["selected_object"] = selected_object.lower()

        elif line.startswith("Cropping Box Coordinates:"):
            coords = line.split(": ")[1].strip()[1:-1]
            result["cropping_box"] = tuple(map(int, coords.split(", ")))
        elif line.startswith("Object:"):
            obj = {
                "name": line.split(": ")[1].strip().lower(),
                "grasping_score": int(lines[i + 1].split(": ")[1].strip()),
                "material_composition": int(lines[i + 2].split(": ")[1].strip()),
                "surface_texture": int(lines[i + 3].split(": ")[1].strip()),
                "stability_assessment": int(lines[i + 4].split(": ")[1].strip()),
                "centroid_coordinates": tuple(map(int, lines[i + 5].split(": ")[1].strip()[1:-1].split(", "))),
                "preferred_grasping_location": int(lines[i + 6].split(": ")[1].strip())
            }
            result["objects"].append(obj)

    return result


def select_fallback_object(objects):
    sorted_objects = sorted(objects, key=lambda x: x['grasping_score'], reverse=True)
    return sorted_objects[0] if sorted_objects else None

def create_cropping_box_from_boxes(boxes, image_size, margin=20):
    if not boxes:
        return 0, 0, image_size[0], image_size[1]

    x1_min = float('inf')
    y1_min = float('inf')
    x2_max = float('-inf')
    y2_max = float('-inf')

    for box in boxes:
        x1, y1, x2, y2 = box
        if x1 < x1_min:
            x1_min = x1
        if y1 < y1_min:
            y1_min = y1
        if x2 > x2_max:
            x2_max = x2
        if y2 > y2_max:
            y2_max = y2

    x1_min = max(0, x1_min - margin)
    y1_min = max(0, y1_min - margin)
    x2_max = min(image_size[0], x2_max + margin)
    y2_max = min(image_size[1], y2_max + margin)

    return int(x1_min), int(y1_min), int(x2_max), int(y2_max)


def crop_pointcloud(pcd, cropping_box, color_image, depth_image):
    x1, y1, x2, y2 = cropping_box
    depth_crop = depth_image[y1:y2, x1:x2]
    color_crop = color_image[y1:y2, x1:x2]
    mask = (depth_crop > 0)
    points = []
    colors = []

    for i in range(depth_crop.shape[0]):
        for j in range(depth_crop.shape[1]):
            if mask[i, j]:
                x = j + x1
                y = i + y1
                z = depth_crop[i, j]
                points.append((x, y, z))
                colors.append(color_crop[i, j] / 255.0)

    points = np.array(points)
    colors = np.array(colors)

    full_pcd = o3d.geometry.PointCloud()
    full_pcd.points = o3d.utility.Vector3dVector(points)
    full_pcd.colors = o3d.utility.Vector3dVector(colors)

    return full_pcd


def save_point_cloud_json(pcd, output_path):
    """Save point cloud as JSON for web visualization"""
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    
    # Downsample if too many points for web performance
    max_points = 50000
    if len(points) > max_points:
        indices = np.random.choice(len(points), max_points, replace=False)
        points = points[indices]
        if len(colors) > 0:
            colors = colors[indices]
        logging.info(f"Downsampled point cloud from {len(np.asarray(pcd.points))} to {max_points} points for web visualization")
    
    # Create JSON data
    point_data = {
        "points": points.tolist(),
        "colors": colors.tolist() if len(colors) > 0 else [],
        "count": len(points),
        "original_count": len(np.asarray(pcd.points))
    }
    
    import json
    with open(output_path, 'w') as f:
        json.dump(point_data, f)
    
    logging.info(f"Point cloud JSON saved to: {output_path}")
    return output_path


def save_gripper_json(gripper_mesh, output_path):
    """Save gripper mesh as JSON for web visualization"""
    try:
        vertices = np.asarray(gripper_mesh.vertices)
        triangles = np.asarray(gripper_mesh.triangles)
        
        # Create JSON data
        mesh_data = {
            "vertices": vertices.tolist(),
            "faces": triangles.tolist(),
            "vertex_count": len(vertices),
            "face_count": len(triangles)
        }
        
        import json
        with open(output_path, 'w') as f:
            json.dump(mesh_data, f)
        
        logging.info(f"Gripper mesh JSON saved to: {output_path}")
        return output_path
    except Exception as e:
        logging.error(f"Error saving gripper JSON: {e}")
        return None


def generate_html_report(output_folder, timestamp_str, input_text, openai_response, final_results, num_grasp_poses, num_bboxes):
    """Generate HTML visualization report for the results"""
    
    # Since we removed the template, just create a simple info file
    import json
    
    # Create a simple summary JSON file instead
    summary = {
        "timestamp": timestamp_str,
        "input_text": input_text,
        "openai_response": openai_response,
        "num_grasp_poses": num_grasp_poses,
        "selected_action_idx": final_results.get('action_index', 'N/A'),
        "num_bboxes": num_bboxes,
        "position_xyz": final_results.get('position_xyz', 'N/A'),
        "rotation_matrix": final_results.get('rotation_matrix', 'N/A'),
        "grasp_depth": final_results.get('depth', 'N/A'),
        "final_results": final_results,
        "input_files": {
            "rgb_image": "input_rgb.png",
            "depth_image": "input_depth.png",
            "depth_visualization": "input_depth_visualization.png",
            "instruction_text": "input_instruction.txt",
            "cropping_box_visualization": "cropping_box_visualization.png"
        }
    }
    
    summary_path = f"{output_folder}/summary.json"
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    
    logging.info(f"Summary saved to: {summary_path}")
    return summary_path


@app.route('/grasp_pose', methods=['POST'])
def get_grasp_pose():
    # Generate timestamp for this request
    timestamp = datetime.datetime.now()
    timestamp_str = timestamp.strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
    
    # Create timestamped output folder
    output_folder = f"outputs/{timestamp_str}"
    os.makedirs(output_folder, exist_ok=True)
    
    data = request.json
    rgb_image_path = data['image_path']
    depth_image_path = data['depth_path']
    text_path = data['text_path']
    
    logging.info(f"Starting grasp pose detection at {timestamp} (ID: {timestamp_str})")
    logging.info(f"Output folder: {output_folder}")


    img_ori = cv2.imread(rgb_image_path)
    img_ori = cv2.cvtColor(img_ori, cv2.COLOR_BGR2RGB)
    depth_ori = np.array(Image.open(depth_image_path))
    with open(text_path, 'r') as file:
        text = file.read()

    # Save input files to output folder for reference
    import shutil
    
    # Save original RGB image
    input_rgb_path = f"{output_folder}/input_rgb.png"
    shutil.copy2(rgb_image_path, input_rgb_path)
    logging.info(f"Input RGB image saved to: {input_rgb_path}")
    
    # Save original depth image
    input_depth_path = f"{output_folder}/input_depth.png"
    shutil.copy2(depth_image_path, input_depth_path)
    logging.info(f"Input depth image saved to: {input_depth_path}")
    
    # Save grasp instruction text
    input_text_path = f"{output_folder}/input_instruction.txt"
    with open(input_text_path, 'w') as f:
        f.write(text)
    logging.info(f"Input instruction saved to: {input_text_path}")
    
    # Also create a depth visualization for easier viewing
    depth_vis = cv2.normalize(depth_ori, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    depth_vis_path = f"{output_folder}/input_depth_visualization.png"
    cv2.imwrite(depth_vis_path, depth_colored)
    logging.info(f"Depth visualization saved to: {depth_vis_path}")

    image_pil = langsamutils.load_image(rgb_image_path)
    base64_image = load_image_as_base64(rgb_image_path)
    input_text = text


    messages = [
        {
            "role": "system",
            "content": (
                "Given a 640x480 input image and the provided instruction, perform the following steps:\n"
                "Target Object Selection:\n"
                "Identify the object in the image that best matches the instruction. If the target object is found, select it as the target object.\n"
                "If the target object is not visible, select the most cost-effective object or object part considering ease of grasping, importance, and safety.\n"
                "If the object has a handle or a part that is easier or safer to grasp, strongly prefer to select that part.\n"
                "Consider the geometric shape of the objects and the gripper's success rate when selecting the target object or object part.\n"
                "Output the name of the selected object or object part as [object:color and name] or [object part:color and name]..\n"
                "Cropping Box Calculation:\n"
                "Calculate a cropping box that includes the target object and all surrounding objects that might be relevant for grasping.\n"
                "Provide the coordinates of the cropping box in the format (top-left x, top-left y, bottom-right x, bottom-right y).\n"
                "Object Properties within Cropping Box:\n"
                "For each object within the cropping box, provide the following properties:\n"
                "Grasping Score: Evaluate the ease or difficulty of grasping the object on a scale from 0 to 100 (0 being extremely difficult, 100 being extremely easy).\n"
                "Material Composition: Evaluate the material composition of the object on a scale from 0 to 100 (0 being extremely weak, 100 being extremely strong).\n"
                "Surface Texture: Evaluate the texture of the object's surface on a scale from 0 to 100 (0 being extremely smooth, 100 being extremely rough).\n"
                "Stability Assessment: Assess the stability of the object on a scale from 0 to 100 (0 being extremely unstable, 100 being extremely stable).\n"
                "Centroid Coordinates: Provide the coordinates (x, y) of the object's center of mass across the entire image.\n"
                "Preferred Grasping Location: Divide the cropping box into a 3x3 grid and return a number from 1 to 9 indicating the preferred grasping location (1 for top-left, 9 for bottom-right).\n"
                "Additionally, consider the preferred grasping location that is most successful for the UR5 robotic arm and gripper.\n"
                "Output should be in the following format:\n"
                "Selected Object/Object Part: [object:color and name] or [object part:color and name]\n"
                "Cropping Box Coordinates: (top-left x, top-left y, bottom-right x, bottom-right y)\n"
                "Objects and Their Properties:\n"
                "Object: [color and name]\n"
                "Grasping Score: [value]\n"
                "Material Composition: [value]\n"
                "Surface Texture: [value]\n"
                "Stability Assessment: [value]\n"
                "Centroid Coordinates: (x, y)\n"
                "Preferred Grasping Location: [value]\n"
                "...\n"
                "Example Output:\n"
                "Selected Object/Object Part: [object:blue ball]\n"
                "Cropping Box Coordinates: (50, 50, 200, 200)\n"
                "Objects and Their Properties:\n"
                "Object: Blue Ball\n"
                "Grasping Score: 90\n"
                "Material Composition: 80\n"
                "Surface Texture: 20\n"
                "Stability Assessment: 95\n"
                "Centroid Coordinates: (125, 125)\n"
                "Preferred Grasping Location: 5\n"
                "Object: Yellow Bottle\n"
                "Grasping Score: 75\n"
                "Material Composition: 70\n"
                "Surface Texture: 30\n"
                "Stability Assessment: 80\n"
                "Centroid Coordinates: (100, 150)\n"
                "Preferred Grasping Location: 3\n"
                "Object: Black and Blue Scissors\n"
                "Grasping Score: 60\n"
                "Material Composition: 85\n"
                "Surface Texture: 40\n"
                "Stability Assessment: 70\n"
                "Centroid Coordinates: (175, 175)\n"
                "Preferred Grasping Location: 7"
            )
        },
        {
            "role": "user",
            "content": [
                {"type": "text", "text": input_text},
                {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{base64_image}"}}
            ]
        }
    ]

    result = {
        "selected_object": None,
        "cropping_box": None,
        "objects": []
    }

    try:
        if not client:
            return jsonify({"error": "OpenAI client not available. Please check your API key."}), 500
            
        response = client.chat.completions.create(
            model="gpt-4o-2024-05-13",
            messages=messages,
            temperature=0,
            max_tokens=713,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0
        )
        output = response.choices[0].message.content
        
        # Save OpenAI response to log file
        openai_response_path = f"{output_folder}/openai_response.txt"
        with open(openai_response_path, "w") as f:
            f.write(output)
        logging.info(f"OpenAI response saved to {openai_response_path}")
        logging.info(f"OpenAI response: {output}")
        
        result = process_grasping_result(output)

        if not result['selected_object']:
            fallback_object = select_fallback_object(result['objects'])
            if fallback_object:
                goal = fallback_object['name']
            else:
                goal = input_text
        else:
            goal = result['selected_object']

        preferred_grasping_location = 5
        for obj in result['objects']:
            if obj['name'] == goal:
                preferred_grasping_location = obj.get('preferred_grasping_location', 5)

        if not result["is_part"]:
            masks, boxes, phrases, logits = langsam_actor.predict(image_pil, goal)
        else:
            # VLP actor is commented out, use langsam_actor as fallback
            # masks, boxes, phrases, logits = vlp_actor.predict(image_path=rgb_image_path, text_prompt=goal)
            masks, boxes, phrases, logits = langsam_actor.predict(image_pil, goal)

        if masks is None or masks.numel() == 0:
            masks, boxes, phrases, logits = langsam_actor.predict(image_pil, input_text)
            if masks is None or masks.numel() == 0:
                masks, boxes, phrases, logits = langsam_actor.predict(image_pil, "object")

        boxes_list = boxes.cpu().numpy().tolist()
        cropping_box = create_cropping_box_from_boxes(boxes_list, (img_ori.shape[1], img_ori.shape[0]))

        # Save cropping box visualization
        crop_vis_path = save_cropping_box_visualization(img_ori, cropping_box, output_folder)
        # Save segmentation results
        langsam_actor.save(masks, boxes, phrases, logits, image_pil)
        logging.info(f"LangSAM segmentation results saved")
        
        bbox_images, bbox_positions = utils.convert_outputnew(image_pil, boxes, phrases, logits, img_ori, depth_ori, preferred_grasping_location)
        
        # Log bbox information
        logging.info(f"Number of detected bounding boxes: {len(boxes) if boxes is not None else 0}")
        logging.info(f"Detected phrases: {phrases}")
        logging.info(f"Bounding box positions: {bbox_positions}")

        endpoint ,pcd = utils.get_and_process_data(cropping_box,img_ori, depth_ori)

        parser = argparse.ArgumentParser('Deformable DETR training and evaluation script', parents=[get_args_parser()])
        args = parser.parse_args([])  # Provide empty list to avoid reading command-line args
        # Override output_dir_grasp to use timestamped folder
        args.output_dir_grasp = output_folder
        grasp_net = grasp_model(args=args,device="cuda" if use_gpu else "cpu", image=img_ori, bbox=bbox_positions, mask=masks, text=input_text)
        gg ,gg_array= grasp_net.forward(endpoint ,pcd )
        grasp_pose_set=gg_array
        
        # Log grasp detection results
        logging.info(f"Number of grasp poses detected: {len(grasp_pose_set) if grasp_pose_set is not None else 0}")
        
        # Check if any grasps were detected
        if grasp_pose_set is None or len(grasp_pose_set) == 0:
            logging.error("No grasp poses detected")
            return jsonify({
                "error": "No grasp poses detected", 
                "details": "The grasp detection model could not find any valid grasp poses for the given object.",
                "timestamp": timestamp_str
            }), 400
        
        # Save grasp poses to numpy file
        grasp_poses_path = f"{output_folder}/grasp_poses.npy"
        np.save(grasp_poses_path, grasp_pose_set)
        logging.info(f"Grasp poses saved to {grasp_poses_path}")
        
        remain_bbox_images, bboxes, pos_bboxes, grasps = utils.preprocess(bbox_images, bbox_positions, grasp_pose_set, (32, 32))

        if bboxes is None:
            return jsonify({"error": "No bounding boxes found"}), 400

        if len(grasp_pose_set) == 1:
            action_idx = 0
        else:
            with torch.no_grad():
                action_idx = select_action(bboxes, pos_bboxes, input_text, grasps)

        action = grasp_pose_set[action_idx]
        
        # Log selected action details
        logging.info(f"Selected action index: {action_idx}")
        logging.info(f"Selected action: {action}")

        chose_xyz = action[-4:-1]
        chose_rot = np.resize(np.expand_dims(action[-13:-4], axis=0), (3, 3))
        dep = action[3]

        # Log final grasp pose results
        logging.info(f"Final grasp position (xyz): {chose_xyz}")
        logging.info(f"Final grasp rotation matrix: {chose_rot}")
        logging.info(f"Final grasp depth: {dep}")
        
        # Save final results to file
        final_results = {
            "action_index": int(action_idx),
            "full_action": action.tolist() if hasattr(action, 'tolist') else action,
            "position_xyz": chose_xyz.tolist() if hasattr(chose_xyz, 'tolist') else chose_xyz,
            "rotation_matrix": chose_rot.tolist() if hasattr(chose_rot, 'tolist') else chose_rot,
            "depth": float(dep) if hasattr(dep, 'item') else dep
        }
        
        import json
        final_results_path = f"{output_folder}/final_grasp_results.json"
        with open(final_results_path, "w") as f:
            json.dump(final_results, f, indent=2)
        logging.info(f"Final grasp results saved to {final_results_path}")

        # Convert ndarrays to lists
        xyz_list = chose_xyz.tolist()
        rot_list = chose_rot.tolist()
        dep_list = dep.tolist()
        grippers = gg.to_open3d_geometry_list()
        chosen_gripper = grippers[action_idx]
        
        # Save point cloud and grasp pose visualization
        pcd_path = f"{output_folder}/point_cloud.ply"
        grasp_path = f"{output_folder}/chosen_grasp.ply"
        o3d.io.write_point_cloud(pcd_path, pcd)
        o3d.io.write_triangle_mesh(grasp_path, chosen_gripper)
        logging.info(f"Point cloud saved to: {pcd_path}")
        logging.info(f"Chosen grasp pose saved to: {grasp_path}")
        
        # Save JSON versions for web visualization
        pcd_json_path = f"{output_folder}/point_cloud.json"
        grasp_json_path = f"{output_folder}/chosen_grasp.json"
        save_point_cloud_json(pcd, pcd_json_path)
        save_gripper_json(chosen_gripper, grasp_json_path)
        
        # Save all grasp poses for visualization
        all_grippers_data = []
        for i, gripper in enumerate(grippers):
            gripper_data = {
                "id": i,
                "selected": i == action_idx,
                "vertices": np.asarray(gripper.vertices).tolist(),
                "faces": np.asarray(gripper.triangles).tolist()
            }
            all_grippers_data.append(gripper_data)
        
        all_grippers_path = f"{output_folder}/all_grasps.json"
        with open(all_grippers_path, 'w') as f:
            json.dump({"grasps": all_grippers_data, "selected_index": action_idx}, f)
        logging.info(f"All grasp poses saved to: {all_grippers_path}")
        
        # Generate summary report
        summary_path = generate_html_report(
            output_folder=output_folder,
            timestamp_str=timestamp_str,
            input_text=text,
            openai_response=output,
            final_results=final_results,
            num_grasp_poses=len(grasp_pose_set) if grasp_pose_set is not None else 0,
            num_bboxes=len(boxes) if boxes is not None else 0
        )

        return jsonify({
                'xyz': xyz_list,
                'rot': rot_list,
                'dep': dep_list,
                'summary_file': summary_path,
                'timestamp': timestamp_str,
                'message': 'Use http://localhost:5010/viewer to visualize results'
            })


    except Exception as e:
        import traceback
        error_details = {
            "error_type": type(e).__name__,
            "error_message": str(e),
            "traceback": traceback.format_exc(),
            "function": "OpenAI API request",
            "request_data": {
                "image_path": rgb_image_path,
                "depth_path": depth_image_path,
                "text_path": text_path,
                "text_content": text[:100] + "..." if len(text) > 100 else text
            }
        }
        logging.error(f"Error with OpenAI API request: {error_details}")
        return jsonify({"error": error_details}), 500


@app.route('/visualize/<timestamp>')
def serve_visualization(timestamp):
    """Serve the HTML visualization for a specific timestamp"""
    try:
        return send_from_directory(f'outputs/{timestamp}', 'results_visualization.html')
    except Exception as e:
        return jsonify({"error": f"Visualization not found for timestamp {timestamp}"}), 404


@app.route('/results/<timestamp>/<filename>')
def serve_result_files(timestamp, filename):
    """Serve result files (images, point clouds, etc.) for a specific timestamp"""
    try:
        return send_from_directory(f'outputs/{timestamp}', filename)
    except Exception as e:
        return jsonify({"error": f"File {filename} not found for timestamp {timestamp}"}), 404


@app.route('/list_results')
def list_results():
    """List all available result timestamps"""
    try:
        outputs_dir = "outputs"
        if not os.path.exists(outputs_dir):
            return jsonify({"timestamps": []})
        
        timestamps = []
        for item in os.listdir(outputs_dir):
            item_path = os.path.join(outputs_dir, item)
            # Check if it's a valid result folder (has point cloud or summary)
            if os.path.isdir(item_path) and (
                os.path.exists(os.path.join(item_path, "point_cloud.ply")) or 
                os.path.exists(os.path.join(item_path, "point_cloud.json")) or
                os.path.exists(os.path.join(item_path, "summary.json"))
            ):
                timestamps.append(item)
        
        timestamps.sort(reverse=True)  # Most recent first
        return jsonify({"timestamps": timestamps})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/viewer')
def serve_results_viewer():
    """Serve the standalone results viewer HTML"""
    try:
        return send_from_directory('.', 'results_viewer.html')
    except Exception as e:
        return jsonify({"error": f"Results viewer not found"}), 404


@app.route('/steps/<timestamp>')
def list_steps(timestamp):
    """List all intermediate steps for a specific result"""
    try:
        result_dir = f"outputs/{timestamp}"
        if not os.path.exists(result_dir):
            # Return empty steps for legacy results instead of 404
            return jsonify({"steps": [], "message": "Legacy result - no intermediate steps available"})
        
        steps = []
        for item in os.listdir(result_dir):
            item_path = os.path.join(result_dir, item)
            if os.path.isdir(item_path) and item.startswith(('01_', '02_', '03_')):
                metadata_path = os.path.join(item_path, 'metadata.json')
                if os.path.exists(metadata_path):
                    with open(metadata_path, 'r') as f:
                        metadata = json.load(f)
                    steps.append({
                        "folder": item,
                        "step_name": metadata.get("step_name", item),
                        "num_grasps": metadata.get("num_grasps", 0),
                        "metadata": metadata
                    })
        
        steps.sort(key=lambda x: x["folder"])
        
        # Return empty steps if none found (legacy result)
        if not steps:
            return jsonify({"steps": [], "message": "Legacy result - no intermediate steps available"})
            
        return jsonify({"steps": steps})
        
    except Exception as e:
        logging.error(f"Error listing steps for {timestamp}: {str(e)}")
        # Return empty steps instead of error for robustness
        return jsonify({"steps": [], "error": str(e)})


@app.route('/step_files/<timestamp>/<step_folder>/<filename>')
def serve_step_files(timestamp, step_folder, filename):
    """Serve files from intermediate steps"""
    try:
        return send_from_directory(f'outputs/{timestamp}/{step_folder}', filename)
    except Exception as e:
        return jsonify({"error": f"File {filename} not found in step {step_folder}"}), 404


@app.route('/convert_ply/<timestamp>/<filename>')
def convert_ply_to_json(timestamp, filename):
    """Convert PLY file to JSON for legacy results"""
    try:
        ply_path = f"outputs/{timestamp}/{filename}"
        
        if not os.path.exists(ply_path):
            logging.warning(f"PLY file not found: {ply_path}")
            return jsonify({"error": f"PLY file {filename} not found at {ply_path}"}), 404
        
        # Load PLY file
        if filename == "cloud.ply":
            try:
                # Load point cloud
                pcd = o3d.io.read_point_cloud(ply_path)
                points = np.asarray(pcd.points)
                colors = np.asarray(pcd.colors)
                
                if len(points) == 0:
                    return jsonify({"error": "PLY file contains no points"}), 400
                
                # Downsample if too many points
                max_points = 30000
                if len(points) > max_points:
                    indices = np.random.choice(len(points), max_points, replace=False)
                    points = points[indices]
                    if len(colors) > 0:
                        colors = colors[indices]
                
                point_data = {
                    "points": points.tolist(),
                    "colors": colors.tolist() if len(colors) > 0 else [],
                    "count": len(points),
                    "original_count": len(np.asarray(pcd.points))
                }
                
                logging.info(f"Successfully converted PLY to JSON: {len(points)} points")
                return jsonify(point_data)
                
            except Exception as ply_error:
                logging.error(f"Error reading PLY file {ply_path}: {str(ply_error)}")
                return jsonify({"error": f"Error reading PLY file: {str(ply_error)}"}), 500
            
        else:
            return jsonify({"error": f"Conversion not supported for {filename}"}), 400
            
    except Exception as e:
        logging.error(f"Error in PLY conversion for {timestamp}/{filename}: {str(e)}")
        return jsonify({"error": f"Error converting PLY: {str(e)}"}), 500


def cleanup():
    """Clean up resources on application shutdown"""
    global client
    if client:
        try:
            # Properly close the OpenAI client
            if hasattr(client, 'close'):
                client.close()
        except Exception as e:
            logging.warning(f"Error during OpenAI client cleanup: {e}")
        finally:
            client = None

if __name__ == "__main__":
    import atexit
    atexit.register(cleanup)
    
    parser = argparse.ArgumentParser('Deformable DETR training and evaluation script', parents=[get_args_parser()])
    args = parser.parse_args()
    if args.output_dir:
        Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    
    try:
        app.run(host='0.0.0.0', port=5010)
    except KeyboardInterrupt:
        logging.info("Application interrupted by user")
    finally:
        cleanup()
