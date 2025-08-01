# Purpose

This version of the PathMaker is an upgrade of the original implementation. The main improvement is the use of a state machine wrapped around the original engine that calculates the power in the three degrees of freedom (forward, strafe, turn) of a mecanum wheel drive.

# Change log

- Moved autoPathList to PathDetails class. Makes more sense.
- Modifications to PathMakerStateMachine, PathManager as well as small updates to PathDetails and Tele_Robot1:
- added autonomous turn control to field centric driving see function PMSM getGamepadInput
- ...now PM variables autonomous_x, autonomous_y, autonomous_a can be set true/false as needed
- ...to control if a field DOF is driver controlled or autonomous
- ...Also added autoLaneKeeping function to PathMakerStateMachine.


# Git Collaborator

Here are the steps to connect, clone, and start working with a GitHub repository as a collaborator:

Note: You do not need to run git init if you are cloning an existing repository. The git clone command automatically initializes a new Git repository in the cloned folder. Use git init only when starting a brand new repository from scratch.

1. Accept the Invitation
Go to your email or GitHub notifications and accept the collaborator invitation.

2. Install Git
Make sure Git is installed on your Windows system.
You can download it from git-scm.com.

3. Authenticate with GitHub

Set up SSH keys (recommended) or use HTTPS with your GitHub credentials.
For SSH:
Generate a key with ssh-keygen, add it to your GitHub account.
Clone the Repository

4. Get the repository URL from GitHub (either SSH or HTTPS).

Open your terminal (Command Prompt, PowerShell, or Git Bash).
Run:
git clone <repo-url>
Example:
git clone https://github.com/username/repo-name.git

5. Navigate to the Project Folder

cd repo-name

6. Configure Git (if needed)

Set your name and email:
git config user.name "Your Name"
git config user.email "your@email.com"

7.a) Create a Branch (optional)
git checkout -b my-feature-branch
7.b) Switch to a branch
Switching to another branch (example): git checkout Roomba
If the branch does not exist locally, fetch all branches first: git fetch origin

8. Start Working

Open the project in your IDE (e.g., Android Studio).
Make changes, commit, and push as needed.
You are now ready to collaborate on the repository.
