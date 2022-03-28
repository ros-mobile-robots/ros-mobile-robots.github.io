# DiffBot Robot

The [`diffbot_robot`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_robot) package is a 
[ROS metapackage](http://wiki.ros.org/Metapackages) that references all related packages to DiffBot.
A metapackage can be created with:

```console
fjp@ubuntu:ros_ws/src/$ catkin_create_pkg diffbot_robot --meta    
Created file diffbot_robot/package.xml
Created file diffbot_robot/CMakeLists.txt
Successfully created files in /home/fjp/git/ros_ws/src/diffbot/diffbot_robot. Please adjust the values in package.xml.
```

To release a package see the [bloom](http://wiki.ros.org/bloom) page and the listed tutorials there. Specifically the following ones:

- To index the package follow the [Indexing Your ROS Repository for Documentation Generation](http://wiki.ros.org/rosdistro/Tutorials/Indexing%20Your%20ROS%20Repository%20for%20Documentation%20Generation).
- Release a package using bloom, see [First Time Release tutorial](http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease).

## Prepare and Create a New Release

The steps to do create a new release are the following

1. `cd` to the `diffbot` repository which contains all the modified packages and make sure there are no uncommitted or untracked changes.
2. Use `catkin_generate_changelog` to create updated changelogs (populated with the previous commit messages):

    ```console
    ➜  diffbot git:(noetic-devel) ✗ catkin_generate_changelog
    Found packages: diffbot_base, diffbot_bringup, diffbot_control, diffbot_description, diffbot_gazebo, diffbot_mbf, diffbot_msgs, diffbot_navigation, diffbot_robot, diffbot_slam
    Querying commit information since latest tag...
    Updating forthcoming section of changelog files...
    - updating 'diffbot_base/CHANGELOG.rst'
    - updating 'diffbot_bringup/CHANGELOG.rst'
    - updating 'diffbot_control/CHANGELOG.rst'
    - updating 'diffbot_description/CHANGELOG.rst'
    - updating 'diffbot_gazebo/CHANGELOG.rst'
    - updating 'diffbot_mbf/CHANGELOG.rst'
    - updating 'diffbot_msgs/CHANGELOG.rst'
    - updating 'diffbot_navigation/CHANGELOG.rst'
    - updating 'diffbot_robot/CHANGELOG.rst'
    - updating 'diffbot_slam/CHANGELOG.rst'
    Done.
    Please review the extracted commit messages and consolidate the changelog entries before committing the files!
    ```
3. Clean up the Changelog: Go through every package's CHANGELOG.rst file and modify the commit messages if needed.
4. Commit the updated CHANGELOG.rst files

    ```console
    ➜  diffbot git:(noetic-devel) ✗ git status
    On branch noetic-devel
    Your branch is up to date with 'origin/noetic-devel'.

    Changes not staged for commit:
      (use "git add <file>..." to update what will be committed)
      (use "git restore <file>..." to discard changes in working directory)
            modified:   diffbot_base/CHANGELOG.rst
            modified:   diffbot_bringup/CHANGELOG.rst
            modified:   diffbot_control/CHANGELOG.rst
            modified:   diffbot_description/CHANGELOG.rst
            modified:   diffbot_gazebo/CHANGELOG.rst
            modified:   diffbot_mbf/CHANGELOG.rst
            modified:   diffbot_msgs/CHANGELOG.rst
            modified:   diffbot_navigation/CHANGELOG.rst
            modified:   diffbot_robot/CHANGELOG.rst
            modified:   diffbot_slam/CHANGELOG.rst

    no changes added to commit (use "git add" and/or "git commit -a")
    ➜  diffbot git:(noetic-devel) ✗ git add .
    ➜  diffbot git:(noetic-devel) ✗ git status
    On branch noetic-devel
    Your branch is up to date with 'origin/noetic-devel'.

    Changes to be committed:
      (use "git restore --staged <file>..." to unstage)
            modified:   diffbot_base/CHANGELOG.rst
            modified:   diffbot_bringup/CHANGELOG.rst
            modified:   diffbot_control/CHANGELOG.rst
            modified:   diffbot_description/CHANGELOG.rst
            modified:   diffbot_gazebo/CHANGELOG.rst
            modified:   diffbot_mbf/CHANGELOG.rst
            modified:   diffbot_msgs/CHANGELOG.rst
            modified:   diffbot_navigation/CHANGELOG.rst
            modified:   diffbot_robot/CHANGELOG.rst
            modified:   diffbot_slam/CHANGELOG.rst
    ➜  diffbot git:(noetic-devel) ✗ git commit -m "prepare release: updating CHANGELOG.rst files"
    [noetic-devel d2849d9] prepare release: updating CHANGELOG.rst files
     10 files changed, 70 insertions(+)
    ```
5. Bump up the package.xml version

    ```console
    ➜  diffbot git:(noetic-devel) catkin_prepare_release --bump minor
    Prepare the source repository for a release.
    Repository type: git
    Found packages: diffbot_base, diffbot_bringup, diffbot_control, diffbot_description, diffbot_gazebo, diffbot_mbf, diffbot_msgs, diffbot_navigation, diffbot_robot, diffbot_slam
    Prepare release of version '1.1.0' [Y/n]?
    Trying to push to remote repository (dry run)...
    Everything up-to-date
    Checking if working copy is clean (no staged changes, no modified files, no untracked files)...
    Rename the forthcoming section of the following packages to version '1.1.0': diffbot_base, diffbot_bringup, diffbot_control, diffbot_description, diffbot_gazebo, diffbot_mbf, diffbot_msgs, diffbot_navigation, diffbot_robot, diffbot_slam
    Bump version of all packages from '1.0.0' to '1.1.0'
    Committing the package.xml files...
    [noetic-devel e7b6c8c] 1.1.0
     20 files changed, 30 insertions(+), 30 deletions(-)
    Creating tag '1.1.0'...
    The following commands will be executed to push the changes and tag to the remote repository:
      /usr/bin/git push origin noetic-devel
      /usr/bin/git push origin 1.1.0
    Execute commands to push the local commits and tags to the remote repository [Y/n]?
    Enumerating objects: 54, done.
    Counting objects: 100% (54/54), done.
    Delta compression using up to 48 threads
    Compressing objects: 100% (32/32), done.
    Writing objects: 100% (32/32), 5.78 KiB | 227.00 KiB/s, done.
    Total 32 (delta 24), reused 0 (delta 0)
    remote: Resolving deltas: 100% (24/24), completed with 21 local objects.
    To https://github.com/ros-mobile-robots/diffbot.git
       d2849d9..e7b6c8c  noetic-devel -> noetic-devel
    Total 0 (delta 0), reused 0 (delta 0)
    To https://github.com/ros-mobile-robots/diffbot.git
     * [new tag]         1.1.0 -> 1.1.0
    The source repository has been released successfully. The next step will be 'bloom-release'.
    ```

As mentioned in the final output, you are now ready to create a [release repository](http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease#Creating_a_Release_Repository) (if not already [done](https://github.com/ros-mobile-robots-release))) and
follow the rest of the steps to [releasing your package](http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease#Creating_a_Release_Repository).
