import os
import subprocess
import re
from pydrake.systems.framework import SystemBase

def DrawAndSaveDiagram(diagram, path = None):

    if not path:
        # Get the current working directory (pwd)
        current_directory = os.getcwd()
        path = os.path.join(current_directory, diagram.get_name())

    # write the graphviz string into the file.
    with open(path, 'w') as out:
        out.write(diagram.GetGraphvizString())

    # use the dot command to convert the string into drawing.
    # The command is `dot -Tps input_file -o output_file`
    path = re.sub(r" ", r"\\ ", path)

    cmd = f"dot -Tsvg {path} -o {path}.svg"
    subprocess.run(cmd, shell=True)

    # Remove Graphviz string file
    cmd = f"rm {path}"
    subprocess.run(cmd, shell=True)
