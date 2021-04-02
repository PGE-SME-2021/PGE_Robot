import os
import sys
from PIL import Image
import plantuml

if __name__=='__main__':
    try:
        file_name = sys.argv[1]
        print(file_name)
        #os.system(F"plantuml {file_name}.iuml")
        diagram = plantuml.PlantUML(
                url='http://www.plantuml.com/plantuml/img/',
                )
        diagram.processes_file(F"{file_name}.iuml")
        #os.system(F"w3m {file_name}.png -o ext_image_viewer=0")
        img = Image.open(F"{file_name}.png")
        img.show()
    except Exception as e:
        print(e)
