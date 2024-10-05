import os
import aspose.threed as threed
from aspose.threed import Scene

# since this is in the same folder

lst_files = os.listdir()
object_folder = [item for item in lst_files if item == "objects"]
if "objects" not in object_folder:
    os.mkdir("objects")
if "stls"  not in lst_files:
    raise

source_file = "models"
lst_files = os.listdir(source_file)
lst_files_stl = [item for item in lst_files if len(item.split(".")) > 1 and item.split(".")[1] == "stl" or item.split(".")[1] == "STL"]
# print(lst_files)
# print(lst_files_stl)
# check if object is there in the list of files

# lets take all the stl files and convert it
for item in lst_files_stl:
    split_item = item.split(".")
    item_file = split_item[0]
    item_ext = split_item[1]
    scene = Scene.from_file(f"{source_file}/{item_file}.{item_ext}")
    options = threed.formats.ObjSaveOptions()
    scene.save(f"objects/{item_file}.obj", options)
