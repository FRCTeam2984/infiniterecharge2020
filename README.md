# infinterecharge2020
Vikings Robotics's code for InfiniteRecharge 2019

# Basic setup
1. Install python 3.5 or newer
2. Install git
3. Install pip
4. Fork the repository that is owned by `frcteam2984`
5. Clone your forked repository to any directory
6. While in that directory, run `git remote add upstream https://github.com/FRCTeam2984/infinterecharge2020.git`

# Install dev environment
Ensure that you are in a virtual environment before running this
1. `[pip executable] install -r requirements.txt`
2. Open vscode and install Python extension
3. Run `code .`

# Install on robot
Ensure that you are in a virtual environment before running this
1. Run `./installer`
2. When it asks to connect to the robot, disconneect from your current network and connect to the robot's radio

# Deploy to robot
1. Run `[python executable] robot.py deploy`

# Run unit tests
1. Run `[python executable] robot.py test`

# Programming
* You MUST follow the conventions listed in `CONVENTIONS.md` in order for your pull request to be accepted
