import subprocess

class Run_process:
    def __init__(self, path,proc_name):
        """
        :param path - path to file
        :param proc_name - object file name to run
        """
        self.command = path+proc_name 
        
        self.process = None
        self.proc_name = proc_name

    def runobject(self):
        try:
            self.process = subprocess.Popen(self.command)
            print(f"Started process '{self.proc_name}' with PID: {self.process.pid}")
        except Exception as e:
            print(f"Failed to start process '{self.proc_name}':", e)

    def stopobject(self):
        try:
            subprocess.run(["pkill", self.proc_name], check=True)
            return (f"Stopped process '{self.proc_name}'.")
        except subprocess.CalledProcessError:
            return (f"No process named '{self.proc_name}' was running.")
        except Exception as e:
            return (f"Error stopping process '{self.proc_name}':", e)



