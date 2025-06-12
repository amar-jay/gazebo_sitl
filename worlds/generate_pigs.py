import random
import xml.dom.minidom as md

def generate_pig(index, x, y, rotation):
    """Generate XML string for a pig with given parameters"""
    pig_template = f'''
      <visual name="pig_visual{index}">
        <pose degrees="true">{x} {y} 0 90 0 {rotation}</pose>
        <cast_shadows>true</cast_shadows>
        <geometry>
          <mesh>
            <uri>model://farm/meshes/pig_low.obj</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>1.0 0.6 0.8 1</ambient>   <!-- soft pink ambient -->
          <diffuse>1.0 0.6 0.8 1</diffuse>   <!-- soft pink diffuse -->
          <specular>0.2 0.2 0.2 1</specular> <!-- low specular shine -->
          <emissive>0 0 0 1</emissive>       <!-- no glow -->
        </material>
      </visual>
      <collision name="pig_collision{index}">
        <pose degrees="true">{x} {y} 0 90 0 {rotation}</pose>
        <geometry>
          <mesh>
            <uri>model://farm/meshes/pig_low.obj</uri>
          </mesh>
        </geometry>
      </collision>'''
    return pig_template

def main():
    num_pigs = 10  # Change this to create more or fewer pigs
    
    # Generate pig elements with random positions and orientations
    all_pigs = ""
    for i in range(num_pigs):
        # Random position within 10x10 area
        x = round(random.uniform(-5, 5), 2)
        y = round(random.uniform(-5, 5), 2)
        # Random rotation 0-359 degrees
        rotation = round(random.uniform(0, 359), 1)
        
        all_pigs += generate_pig(i, x, y, rotation)
    
    # Print the formatted XML
    print("<!-- Copy this XML into your farm.sdf file under the farm_animals model's link element -->")
    print(all_pigs)
    
    # Optional: Directly write to a file
    with open("pig_elements.xml", "w") as f:
        f.write(all_pigs)
    print(f"\nSaved pig elements to pig_elements.xml")

if __name__ == "__main__":
    main()