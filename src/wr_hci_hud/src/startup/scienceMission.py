import subprocess

def run_shell_command(command):
    try:
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Command output:", result.stdout.decode())
    except subprocess.CalledProcessError as e:
        print("Error occurred:", e.stderr.decode())

# Example usage
def main():
    print("Running a shell command")
    command = "dir"
    run_shell_command(command)
