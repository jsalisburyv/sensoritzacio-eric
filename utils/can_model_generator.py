import json

# Load can data from JSON
with open('cans.json', 'r') as file:
    data = json.load(file)

# Generate Gazebo XML snippet
can_snippet = ""
for can in data['cans']:
    name = can['name']
    x, y, z = can['position']
    can_snippet += f"""
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Beer</uri>
      <name>{name}</name>
      <pose>{x} {y} {z} 0 0 0</pose>
    </include>
    """

# Save to a file or print
print(can_snippet)