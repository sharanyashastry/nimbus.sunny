import os
import subprocess
import re
from pydrake.systems.framework import SystemBase


import socket
from pydrake.geometry import Meshcat
import time
import signal

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



class MeshcatUtils:
    def __init__(self, port=7000):
        self.port = port
        self.meshcat = None  # Initialize to store the Meshcat instance
        self.meshcat_url = None  # Initialize to store the Meshcat URL
        self._start_or_restart_meshcat()

    def _kill_python_process_on_port(port):
        """Kill the Python process on the specified port."""
        try:
            # Find the Python process on the given port and kill it
            pid = subprocess.check_output(f"lsof -t -i :{port} -sTCP:LISTEN -a -c python", shell=True).strip()
            if pid:
                print(f"Killing Python process {pid} on port {port}...")
                os.kill(int(pid), signal.SIGTERM)  # Gracefully terminate the process
                print(f"Python process {pid} killed successfully.")
        except subprocess.CalledProcessError:
            print(f"No Python process found on port {port}.")
        except Exception as e:
            print(f"Error killing Python process on port {port}: {e}")

    def _is_port_free(self, port):
        """Check if the given port is free."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("localhost", port))
                return True  # Port is free
            except OSError:
                return False  # Port is in use

    def _start_or_restart_meshcat(self):
        """Start Meshcat and handle any existing processes on the port."""
        self._kill_python_process_on_port(self.port)  # Kill any existing Python process on the port

        # Wait a moment to ensure the port is released before starting a new Meshcat instance
        time.sleep(1)  # Add a small delay to ensure the port is fully released
        
        # Ensure no processes are holding the port using the alternative method
        if self._is_port_free(self.port):
            print(f"Port {self.port} is free. Proceeding with Meshcat startup.")
        else:
            print(f"Port {self.port} is still in use. Retrying...")
            return

        # Start Meshcat on the specified port
        try:
            print(f"Starting Meshcat instance on port {self.port}...")
            self.meshcat = Meshcat(port=self.port)
            self.meshcat_url = f"http://localhost:{self.port}"  # Grab the Meshcat URL
            print(f"Meshcat started on {self.meshcat_url}.")
        except Exception as e:
            print(f"Error starting Meshcat on port {self.port}: {e}")
            self.meshcat = None

    def get_meshcat_url(self):
        """Return the URL of the running Meshcat instance."""
        return self.meshcat_url


if __name__ == '__main__':
    for i in range(30):
        MeshcatUtils._kill_python_process_on_port(7000 + i)
