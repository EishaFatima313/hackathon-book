---
sidebar_position: 6
---

# Service Tutorial

In this tutorial, you'll create ROS 2 services and clients for request/response communication.

---

## What You'll Build

A service that converts temperatures between Celsius and Fahrenheit.

```
┌──────────────┐      Request       ┌──────────────┐
│   Client     │ ──────────────────→│   Server     │
│  (Converter) │  {value: 25,       │ (Calculator) │
│              │   from: "C",       │              │
│              │   to: "F"}         │              │
│              │ ←──────────────────│              │
│              │  Response          │              │
│              │  {result: 77.0}    │              │
└──────────────┘                    └──────────────┘
```

---

## Step 1: Create Your Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python temp_converter
```

---

## Step 2: Create a Custom Service Definition

Create the directory for service files:

```bash
cd temp_converter
mkdir -p srv
```

Create `temp_converter/srv/TemperatureConvert.srv`:

```
# TemperatureConvert.srv
# Convert temperature between Celsius and Fahrenheit

float32 value       # Input temperature value
string from_unit    # Source unit: "C" or "F"
string to_unit      # Target unit: "C" or "F"
---
float32 result      # Converted temperature
string formula      # Formula used for conversion
```

---

## Step 3: Update package.xml

Add dependencies for message generation:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001"?>
<package format="3">
  <name>temp_converter</name>
  <version>0.0.0</version>
  <description>Temperature conversion service tutorial</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Step 4: Update CMakeLists.txt

Edit `temp_converter/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(temp_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TemperatureConvert.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  temp_converter/converter_server.py
  temp_converter/converter_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

---

## Step 5: Write the Service Server

Create `temp_converter/temp_converter/converter_server.py`:

```python
# converter_server.py
# Temperature conversion service server

import rclpy
from rclpy.node import Node
from temp_converter.srv import TemperatureConvert

class TemperatureConverterServer(Node):
    """Service server for temperature conversion."""
    
    def __init__(self):
        super().__init__('temperature_converter_server')
        
        # Create service
        self.service = self.create_service(
            TemperatureConvert,
            '/convert_temperature',
            self.convert_callback
        )
        
        self.get_logger().info('Temperature Converter Service Ready')
        self.get_logger().info('Service: /convert_temperature')
        
    def convert_callback(self, request, response):
        """Handle conversion requests."""
        
        value = request.value
        from_unit = request.from_unit.upper()
        to_unit = request.to_unit.upper()
        
        # Conversion logic
        if from_unit == to_unit:
            # Same unit, no conversion needed
            response.result = value
            response.formula = f"{value} {from_unit} = {value} {to_unit}"
            
        elif from_unit == 'C' and to_unit == 'F':
            # Celsius to Fahrenheit: F = C × 9/5 + 32
            response.result = value * 9/5 + 32
            response.formula = f"F = C × 9/5 + 32"
            
        elif from_unit == 'F' and to_unit == 'C':
            # Fahrenheit to Celsius: C = (F - 32) × 5/9
            response.result = (value - 32) * 5/9
            response.formula = f"C = (F - 32) × 5/9"
            
        else:
            # Invalid conversion
            self.get_logger().error(
                f'Invalid units: {from_unit} to {to_unit}'
            )
            response.result = 0.0
            response.formula = "Error: Invalid units"
        
        self.get_logger().info(
            f'Converted {value}{from_unit} → {response.result:.2f}{to_unit}'
        )
        
        return response

def main(args=None):
    rclpy.init(args=args)
    server = TemperatureConverterServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 6: Write the Service Client

Create `temp_converter/temp_converter/converter_client.py`:

```python
# converter_client.py
# Temperature conversion service client

import rclpy
from rclpy.node import Node
from temp_converter.srv import TemperatureConvert

class TemperatureConverterClient(Node):
    """Service client for temperature conversion."""
    
    def __init__(self):
        super().__init__('temperature_converter_client')
        
        # Create client
        self.client = self.create_client(
            TemperatureConvert,
            '/convert_temperature'
        )
        
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        self.get_logger().info('Service connected!')
    
    def send_request(self, value, from_unit, to_unit):
        """Send a conversion request."""
        
        # Create request
        request = TemperatureConvert.Request()
        request.value = float(value)
        request.from_unit = from_unit
        request.to_unit = to_unit
        
        # Send request asynchronously
        self.get_logger().info(
            f'Requesting: {value}{from_unit} → {to_unit}'
        )
        
        future = self.client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        
        # Process response
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Result: {response.result:.2f} {to_unit}'
            )
            self.get_logger().info(f'Formula: {response.formula}')
            return response.result
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = TemperatureConverterClient()
    
    # Example conversions
    client.send_request(25, 'C', 'F')   # 25°C → °F
    client.send_request(77, 'F', 'C')   # 77°F → °C
    client.send_request(0, 'C', 'F')    # 0°C → °F
    client.send_request(100, 'C', 'F')  # 100°C → °F
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 7: Update setup.py

Edit `temp_converter/setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'temp_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='Temperature conversion service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'converter_server = temp_converter.converter_server:main',
            'converter_client = temp_converter.converter_client:main',
        ],
    },
)
```

---

## Step 8: Build and Run

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select temp_converter

# Source
source install/setup.bash
```

### Run the Server

**Terminal 1:**
```bash
ros2 run temp_converter converter_server
```

### Run the Client

**Terminal 2:**
```bash
ros2 run temp_converter converter_client
```

Expected output:

```
[INFO] [temperature_converter_client]: Service connected!
[INFO] [temperature_converter_client]: Requesting: 25C → F
[INFO] [temperature_converter_client]: Result: 77.00 F
[INFO] [temperature_converter_client]: Formula: F = C × 9/5 + 32
[INFO] [temperature_converter_client]: Requesting: 77F → C
[INFO] [temperature_converter_client]: Result: 25.00 C
[INFO] [temperature_converter_client]: Formula: C = (F - 32) × 5/9
```

---

## Using the Service from CLI

You can also call the service directly:

```bash
# Call the service
ros2 service call /convert_temperature temp_converter/srv/TemperatureConvert "{value: 25.0, from_unit: 'C', to_unit: 'F'}"

# List available services
ros2 service list

# Get service type
ros2 service type /convert_temperature
```

---

## Understanding Service Communication

```
┌─────────────────────────────────────────────────────────────────┐
│                    Service Communication                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Client                          Server                        │
│   ┌─────────┐                     ┌─────────┐                  │
│   │  Send   │ ───── Request ────→ │ Process │                  │
│   │ Request │                     │ Request │                  │
│   └─────────┘                     └────┬────┘                  │
│        ↑                               │                        │
│        │                               │                        │
│        │         Response              │                        │
│        └───────────────────────────────┘                        │
│                                                                  │
│   Key Characteristics:                                          │
│   • Synchronous (client waits for response)                     │
│   • One-to-one communication                                    │
│   • Request/Response pattern                                    │
│   • Good for queries and commands                               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Try It: Extensions

### Challenge 1: Add Kelvin Support

Extend the service to support Kelvin conversions:

<details>
<summary>Solution Hints</summary>

- Add 'K' as a valid unit
- Add conversion formulas:
  - C → K: K = C + 273.15
  - K → C: C = K - 273.15
  - F → K: K = (F - 32) × 5/9 + 273.15
  - K → F: F = (K - 273.15) × 9/5 + 32

</details>

### Challenge 2: Interactive Client

Create a client that accepts user input:

<details>
<summary>Solution Hints</summary>

```python
def interactive_client():
    print("Temperature Converter (type 'quit' to exit)")
    
    while True:
        value = input("Enter temperature value: ")
        if value.lower() == 'quit':
            break
            
        from_unit = input("From unit (C/F): ")
        to_unit = input("To unit (C/F): ")
        
        # Send request...
```

</details>

---

## Services vs Topics

| Feature | Topics | Services |
|---------|--------|----------|
| **Pattern** | Publish/Subscribe | Request/Response |
| **Direction** | One-to-many | One-to-one |
| **Timing** | Asynchronous | Synchronous |
| **Use Case** | Streaming data | Queries, commands |
| **Example** | Sensor data | "Get parameter" |

---

## What's Next?

Learn about [Custom Messages](./custom-messages.md) to define your own data structures!
