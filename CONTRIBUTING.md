# Contributing to Eagleye
## Repository branch structure

- main-ros2  
This is the latest stable version of Eagleye ROS2. It is updated every three months. 
- develop-ros2  
This is a branch to reflect changes in develop-ros1 to ros2 as well.  
If contributors wish to add functionality to eagleye, they should create a new branch from this develop branch and submit a pull request.
- main-ros1  
This is the latest stable version of Eagleye ROS1. Public development at ros1 ended in May 2023.

- feature/<feature_name>  
Branches for new features and improvements
- fix/<bug_name>  
Branch for fixing non-hotfix bugs
- hotfix/<bug_name>  
Branch for fixing serious bugs in main branch
## Pull Request
When making a pull request, please describe what the pull request is about and how the reviewer can verify that it works.  
If you find after submitting a PR that it cannot be merged because it needs to be corrected, add [WIP] to the title of the PR and remove [WIP] from the title as soon as the correction is complete.
## Coding Rule
Please refer to the coding rules in the [ROS developer's guide](http://wiki.ros.org/DevelopersGuide)