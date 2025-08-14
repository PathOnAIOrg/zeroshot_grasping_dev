# ThinkGrasp API Reference

## Server Endpoints

### `POST /grasp_pose`
Main grasp detection endpoint.

**Request Body:**
```json
{
  "image_path": "/absolute/path/to/rgb.png",
  "depth_path": "/absolute/path/to/depth.png",
  "text_path": "/absolute/path/to/instruction.txt"
}
```

**Response (200 OK):**
```json
{
  "xyz": [x, y, z],              // 3D position
  "rot": [[...], [...], [...]],  // 3x3 rotation matrix
  "dep": 0.04,                   // Grasp depth
  "timestamp": "20250814_123456_789",
  "summary_file": "outputs/.../summary.json",
  "message": "Use http://localhost:5010/viewer to visualize results"
}
```

**Error Response (400/500):**
```json
{
  "error": "Error description",
  "details": "Additional information"
}
```

### `GET /viewer`
Web-based result visualization interface.

### `GET /list_results`
List all available results.

**Response:**
```json
{
  "timestamps": [
    "20250814_123456_789",
    "20250814_123457_890"
  ]
}
```

### `GET /results/<timestamp>/<filename>`
Retrieve specific result file.

### `GET /steps/<timestamp>`
Get processing steps for a result.

**Response:**
```json
{
  "steps": [
    {
      "folder": "01_initial_grasps",
      "step_name": "Initial Grasps",
      "num_grasps": 100
    }
  ]
}
```

## Client Usage

### Simple Client

```bash
python simple_grasp_client.py <rgb> <depth> <instruction> [options]
```

**Options:**
- `--host HOST`: API server host (default: localhost)
- `--port PORT`: API server port (default: 5010)
- `--json`: Output raw JSON response
- `--save FILE`: Save results to JSON file

**Examples:**
```bash
# Basic usage
python simple_grasp_client.py rgb.png depth.png "grasp the cup"

# Remote server
python simple_grasp_client.py rgb.png depth.png "grasp" --host 192.168.1.100

# Save results
python simple_grasp_client.py rgb.png depth.png "grasp" --save output.json

# JSON output
python simple_grasp_client.py rgb.png depth.png "grasp" --json
```

### Test Client

```bash
python test_thinkgrasp_api.py <rgb> <depth> <instruction> [--host HOST] [--port PORT]
```

## Python Integration

### Basic Usage
```python
import requests
import json

def call_thinkgrasp(rgb_path, depth_path, instruction):
    # Write instruction to file
    with open('/tmp/instruction.txt', 'w') as f:
        f.write(instruction)
    
    # Prepare request
    payload = {
        'image_path': rgb_path,
        'depth_path': depth_path,
        'text_path': '/tmp/instruction.txt'
    }
    
    # Call API
    response = requests.post(
        'http://localhost:5010/grasp_pose',
        json=payload
    )
    
    if response.status_code == 200:
        return response.json()
    else:
        raise Exception(f"API error: {response.text}")

# Example usage
result = call_thinkgrasp(
    '/path/to/rgb.png',
    '/path/to/depth.png',
    'grasp the red object'
)

print(f"Position: {result['xyz']}")
print(f"Rotation: {result['rot']}")
print(f"Depth: {result['dep']}")
```

### Advanced Usage with Error Handling
```python
import requests
import time

class ThinkGraspClient:
    def __init__(self, host='localhost', port=5010):
        self.base_url = f"http://{host}:{port}"
        
    def detect_grasp(self, rgb_path, depth_path, instruction):
        """Detect grasp with retry logic"""
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                # Write instruction
                with open('/tmp/inst.txt', 'w') as f:
                    f.write(instruction)
                
                # Call API
                response = requests.post(
                    f"{self.base_url}/grasp_pose",
                    json={
                        'image_path': rgb_path,
                        'depth_path': depth_path,
                        'text_path': '/tmp/inst.txt'
                    },
                    timeout=60
                )
                
                if response.status_code == 200:
                    return response.json()
                else:
                    print(f"Attempt {attempt + 1} failed: {response.status_code}")
                    
            except requests.exceptions.RequestException as e:
                print(f"Attempt {attempt + 1} error: {e}")
                
            if attempt < max_retries - 1:
                time.sleep(2)  # Wait before retry
                
        raise Exception("Failed after all retries")
    
    def list_results(self):
        """Get list of all results"""
        response = requests.get(f"{self.base_url}/list_results")
        return response.json()['timestamps']
    
    def get_result_file(self, timestamp, filename):
        """Download specific result file"""
        response = requests.get(
            f"{self.base_url}/results/{timestamp}/{filename}"
        )
        return response.content

# Usage
client = ThinkGraspClient()
result = client.detect_grasp('rgb.png', 'depth.png', 'grasp the object')
print(f"Timestamp: {result['timestamp']}")
```

## Data Formats

### Input Requirements

**RGB Image:**
- Format: PNG recommended
- Resolution: Any (will be processed)
- Color space: RGB

**Depth Image:**
- Format: PNG (16-bit recommended)
- Alignment: Must align with RGB
- Units: Millimeters or meters

**Instruction Text:**
- Type: Plain text string
- Length: 1-100 characters
- Language: English

### Output Formats

**Grasp Pose:**
- Position: 3D coordinates [x, y, z]
- Orientation: 3x3 rotation matrix
- Depth: Scalar value for gripper depth

**Point Cloud:**
- PLY format for compatibility
- JSON format for web visualization
- Includes colors if available

**Visualization:**
- HTML with Three.js
- Interactive 3D controls
- Multi-panel views

## Error Codes

- **200**: Success
- **400**: Bad request (invalid input)
- **404**: Resource not found
- **500**: Server error
- **503**: Service unavailable

## Rate Limits

- No built-in rate limiting
- Recommended: 1 request per second
- Processing time: 5-30 seconds typical

## Best Practices

1. **Image Quality**: Use well-lit, clear images
2. **Depth Alignment**: Ensure RGB-depth alignment
3. **File Paths**: Use absolute paths
4. **Error Handling**: Implement retry logic
5. **Timeouts**: Set appropriate timeouts (60s recommended)