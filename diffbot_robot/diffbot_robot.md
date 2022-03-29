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

## Install Bloom

[Bloom](http://ros-infrastructure.github.io/bloom/) will be used to release our packages.

```console
➜  diffbot git:(noetic-devel) sudo apt-get install python3-bloom
Reading package lists... Done
Building dependency tree
Reading state information... Done
The following package was automatically installed and is no longer required:
  libfwupdplugin1
Use 'sudo apt autoremove' to remove it.
The following NEW packages will be installed:
  python3-bloom
0 upgraded, 1 newly installed, 0 to remove and 0 not upgraded.
Need to get 68.4 kB of archives.
After this operation, 519 kB of additional disk space will be used.
Get:1 http://packages.ros.org/ros/ubuntu focal/main amd64 python3-bloom all 0.10.7-100 [68.4 kB]
Fetched 68.4 kB in 2s (38.9 kB/s)
Selecting previously unselected package python3-bloom.
(Reading database ... 152404 files and directories currently installed.)
Preparing to unpack .../python3-bloom_0.10.7-100_all.deb ...
Unpacking python3-bloom (0.10.7-100) ...
Setting up python3-bloom (0.10.7-100) ...
```

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

## Releasing your Package for the first time

````console
➜  diffbot git:(noetic-devel) bloom-release --rosdistro noetic --track noetic diffbot --edit
ROS Distro index file associate with commit '725def91e71e2e1a9520416feb916c802ed75314'
New ROS Distro index url: 'https://raw.githubusercontent.com/ros/rosdistro/725def91e71e2e1a9520416feb916c802ed75314/index-v4.yaml'
Specified repository 'diffbot' is not in the distribution file located at 'https://raw.githubusercontent.com/ros/rosdistro/725def91e71e2e1a9520416feb916c802ed75314/noetic/distribution.yaml'
Did you mean one of these: 'audibot', 'ifopt', 'iotbot'?
Could not determine release repository url for repository 'diffbot' of distro 'noetic'
You can continue the release process by manually specifying the location of the RELEASE repository.
To be clear this is the url of the RELEASE repository not the upstream repository.
For release repositories on GitHub, you should provide the `https://` url which should end in `.git`.
Here is the url for a typical release repository on GitHub: https://github.com/ros-gbp/rviz-release.git
==> Looking for a release of this repository in a different distribution...
No reasonable default release repository url could be determined from previous releases.
Release repository url [press enter to abort]: https://github.com/ros-mobile-robots-release/diffbot-release.git
==> Fetching 'diffbot' repository from 'https://github.com/ros-mobile-robots-release/diffbot-release.git'
Cloning into '/tmp/tmpn6xiovfa'...
remote: Enumerating objects: 3, done.
remote: Counting objects: 100% (3/3), done.
remote: Total 3 (delta 0), reused 0 (delta 0), pack-reused 0
Unpacking objects: 100% (3/3), 592 bytes | 592.00 KiB/s, done.
WARNING [vcstools] Command failed: 'git checkout master'
 run at: '/tmp/tmpn6xiovfa'
 errcode: 1:
error: pathspec 'master' did not match any file(s) known to git
[/vcstools]
Creating 'master' branch.
Creating track 'noetic'...
Repository Name:
  <name>
    Name of the repository (used in the archive name)
  upstream
    Default value, leave this as upstream if you are unsure
  ['upstream']: diffbot
Upstream Repository URI:
  <uri>
    Any valid URI. This variable can be templated, for example an svn url
    can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
    where the :{version} token will be replaced with the version for this release.
  [None]: https://github.com/ros-mobile-robots/diffbot.git
Upstream VCS Type:
  git
    Upstream URI is a git repository
  hg
    Upstream URI is a hg repository
  svn
    Upstream URI is a svn repository
  tar
    Upstream URI is a tarball
  ['git']:
Version:
  :{auto}
    This means the version will be guessed from the devel branch.
    This means that the devel branch must be set, the devel branch must exist,
    and there must be a valid package.xml in the upstream devel branch.
  :{ask}
    This means that the user will be prompted for the version each release.
    This also means that the upstream devel will be ignored.
  <version>
    This will be the version used.
    It must be updated for each new upstream version.
  [':{auto}']:
Release Tag:
  :{version}
    This means that the release tag will match the :{version} tag.
    This can be further templated, for example: "foo-:{version}" or "v:{version}"

    This can describe any vcs reference. For git that means {tag, branch, hash},
    for hg that means {tag, branch, hash}, for svn that means a revision number.
    For tar this value doubles as the sub directory (if the repository is
    in foo/ of the tar ball, putting foo here will cause the contents of
    foo/ to be imported to upstream instead of foo itself).
  :{ask}
    This means the user will be prompted for the release tag on each release.
  :{none}
    For svn and tar only you can set the release tag to :{none}, so that
    it is ignored.  For svn this means no revision number is used.
  [':{version}']:
Upstream Devel Branch:
  <vcs reference>
    Branch in upstream repository on which to search for the version.
    This is used only when version is set to ':{auto}'.
  [None]:
ROS Distro:
  <ROS distro>
    This can be any valid ROS distro, e.g. indigo, kinetic, lunar, melodic
  ['noetic']:
Patches Directory:
  <path in bloom branch>
    This can be any valid relative path in the bloom branch. The contents
    of this folder will be overlaid onto the upstream branch after each
    import-upstream.  Additionally, any package.xml files found in the
    overlay will have the :{version} string replaced with the current
    version being released.
  :{none}
    Use this if you want to disable overlaying of files.
  [None]:
Release Repository Push URL:
  <url>
    (optional) Used when pushing to remote release repositories. This is only
    needed when the release uri which is in the rosdistro file is not writable.
    This is useful, for example, when a releaser would like to use a ssh url
    to push rather than a https:// url.
  :{none}
    This indicates that the default release url should be used.
  [None]:
Created 'noetic' track.
==> Testing for push permission on release repository
==> git remote -v
origin  https://github.com/ros-mobile-robots-release/diffbot-release.git (fetch)
origin  https://github.com/ros-mobile-robots-release/diffbot-release.git (push)
==> git push --dry-run
Everything up-to-date
==> Releasing 'diffbot' using release track 'noetic'
==> git-bloom-release noetic
Processing release track settings for 'noetic'
Checking upstream devel branch '<default>' for package.xml(s)
Cloning into '/tmp/tmpa99ewrj1/upstream'...
remote: Enumerating objects: 3242, done.
remote: Counting objects: 100% (974/974), done.
remote: Compressing objects: 100% (557/557), done.
remote: Total 3242 (delta 554), reused 732 (delta 359), pack-reused 2268
Receiving objects: 100% (3242/3242), 8.26 MiB | 1.99 MiB/s, done.
Resolving deltas: 100% (1801/1801), done.
Looking for packages in 'noetic-devel' branch... found 10 packages.
Detected version '1.1.0' from package(s): ['diffbot_navigation', 'diffbot_control', 'diffbot_robot', 'diffbot_bringup', 'diffbot_slam', 'diffbot_gazebo', 'diffbot_msgs', 'diffbot_mbf', 'diffbot_description', 'diffbot_base']

Executing release track 'noetic'
==> bloom-export-upstream /tmp/tmpa99ewrj1/upstream git --tag 1.1.0 --display-uri https://github.com/ros-mobile-robots/diffbot.git --name diffbot --output-dir /tmp/tmpig6hnbdi
Checking out repository at 'https://github.com/ros-mobile-robots/diffbot.git' to reference '1.1.0'.
Exporting to archive: '/tmp/tmpig6hnbdi/diffbot-1.1.0.tar.gz'
md5: ea1e312dd269bac40fe06fcc2f7aa323

==> git-bloom-import-upstream /tmp/tmpig6hnbdi/diffbot-1.1.0.tar.gz  --release-version 1.1.0 --replace
Creating upstream branch.
Importing archive into upstream branch...
Creating tag: 'upstream/1.1.0'
I'm happy.  You should be too.

==> git-bloom-generate -y rosrelease noetic --source upstream -i 1
Releasing packages: ['diffbot_navigation', 'diffbot_control', 'diffbot_robot', 'diffbot_bringup', 'diffbot_slam', 'diffbot_gazebo', 'diffbot_msgs', 'diffbot_mbf', 'diffbot_description', 'diffbot_base']
Releasing package 'diffbot_navigation' for 'noetic' to: 'release/noetic/diffbot_navigation'
Releasing package 'diffbot_control' for 'noetic' to: 'release/noetic/diffbot_control'
Releasing package 'diffbot_robot' for 'noetic' to: 'release/noetic/diffbot_robot'
Releasing package 'diffbot_bringup' for 'noetic' to: 'release/noetic/diffbot_bringup'
Releasing package 'diffbot_slam' for 'noetic' to: 'release/noetic/diffbot_slam'
Releasing package 'diffbot_gazebo' for 'noetic' to: 'release/noetic/diffbot_gazebo'
Releasing package 'diffbot_msgs' for 'noetic' to: 'release/noetic/diffbot_msgs'
Releasing package 'diffbot_mbf' for 'noetic' to: 'release/noetic/diffbot_mbf'
Releasing package 'diffbot_description' for 'noetic' to: 'release/noetic/diffbot_description'
Releasing package 'diffbot_base' for 'noetic' to: 'release/noetic/diffbot_base'

==> git-bloom-generate -y rosdebian --prefix release/noetic noetic -i 1 --os-name ubuntu
Generating source debs for the packages: ['diffbot_mbf', 'diffbot_description', 'diffbot_bringup', 'diffbot_slam', 'diffbot_robot', 'diffbot_base', 'diffbot_navigation', 'diffbot_gazebo', 'diffbot_control', 'diffbot_msgs']
Debian Incremental Version: 1
Debian Distributions: ['focal']
Releasing for rosdistro: noetic

Pre-verifying Debian dependency keys...
Running 'rosdep update'...
All keys are OK

Placing debian template files into 'debian/noetic/diffbot_mbf' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_mbf' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-mbf' has dependencies:
Build and Build Tool Dependencies:
  rosdep key           => focal key
  catkin               => ['ros-noetic-catkin']
ROS Distro index file associate with commit '725def91e71e2e1a9520416feb916c802ed75314'
New ROS Distro index url: 'https://raw.githubusercontent.com/ros/rosdistro/725def91e71e2e1a9520416feb916c802ed75314/index-v4.yaml'
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-mbf_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_mbf' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_description' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_description' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-description' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  joint_state_publisher => ['ros-noetic-joint-state-publisher']
  robot_state_publisher => ['ros-noetic-robot-state-publisher']
  rviz                 => ['ros-noetic-rviz']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  joint_state_publisher => ['ros-noetic-joint-state-publisher']
  robot_state_publisher => ['ros-noetic-robot-state-publisher']
  rviz                 => ['ros-noetic-rviz']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-description_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_description' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_bringup' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_bringup' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-bringup' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  teleop_twist_keyboard => ['ros-noetic-teleop-twist-keyboard']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-bringup_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_bringup' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_slam' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_slam' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-slam' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  gmapping             => ['ros-noetic-gmapping']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-slam_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_slam' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_robot' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_robot' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-robot' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  diffbot_base         => ['ros-noetic-diffbot-base']
  diffbot_bringup      => ['ros-noetic-diffbot-bringup']
  diffbot_control      => ['ros-noetic-diffbot-control']
  diffbot_description  => ['ros-noetic-diffbot-description']
  diffbot_gazebo       => ['ros-noetic-diffbot-gazebo']
  diffbot_navigation   => ['ros-noetic-diffbot-navigation']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-robot_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_robot' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_base' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_base' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-base' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  diagnostic_updater   => ['ros-noetic-diagnostic-updater']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  controller_manager   => ['ros-noetic-controller-manager']
  control_toolbox      => ['ros-noetic-control-toolbox']
  dynamic_reconfigure  => ['ros-noetic-dynamic-reconfigure']
  urdf                 => ['ros-noetic-urdf']
  roscpp               => ['ros-noetic-roscpp']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  diffbot_msgs         => ['ros-noetic-diffbot-msgs']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  rosserial            => ['ros-noetic-rosserial']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  diagnostic_updater   => ['ros-noetic-diagnostic-updater']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  controller_manager   => ['ros-noetic-controller-manager']
  control_toolbox      => ['ros-noetic-control-toolbox']
  dynamic_reconfigure  => ['ros-noetic-dynamic-reconfigure']
  urdf                 => ['ros-noetic-urdf']
  roscpp               => ['ros-noetic-roscpp']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  diffbot_msgs         => ['ros-noetic-diffbot-msgs']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-base_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_base' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_navigation' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_navigation' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-navigation' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  amcl                 => ['ros-noetic-amcl']
  diffbot_bringup      => ['ros-noetic-diffbot-bringup']
  map_server           => ['ros-noetic-map-server']
  move_base            => ['ros-noetic-move-base']
  base_local_planner   => ['ros-noetic-base-local-planner']
  dwa_local_planner    => ['ros-noetic-dwa-local-planner']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  amcl                 => ['ros-noetic-amcl']
  diffbot_bringup      => ['ros-noetic-diffbot-bringup']
  map_server           => ['ros-noetic-map-server']
  move_base            => ['ros-noetic-move-base']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-navigation_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_navigation' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_gazebo' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_gazebo' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-gazebo' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  gazebo_plugins       => ['ros-noetic-gazebo-plugins']
  gazebo_ros           => ['ros-noetic-gazebo-ros']
  gazebo_ros_control   => ['ros-noetic-gazebo-ros-control']
  diffbot_control      => ['ros-noetic-diffbot-control']
  diffbot_description  => ['ros-noetic-diffbot-description']
  xacro                => ['ros-noetic-xacro']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-gazebo_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_gazebo' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_control' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_control' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-control' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  controller_manager   => ['ros-noetic-controller-manager']
  joint_state_controller => ['ros-noetic-joint-state-controller']
  robot_state_publisher => ['ros-noetic-robot-state-publisher']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  roscpp               => ['ros-noetic-roscpp']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  transmission_interface => ['ros-noetic-transmission-interface']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  controller_manager   => ['ros-noetic-controller-manager']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  roscpp               => ['ros-noetic-roscpp']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  transmission_interface => ['ros-noetic-transmission-interface']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-control_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_control' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_msgs' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'focal' debian for package 'diffbot_msgs' at version '1.1.0-1'
####
Generating debian for focal...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-msgs' has dependencies:
Run Dependencies:
  rosdep key           => focal key
  std_msgs             => ['ros-noetic-std-msgs']
  message_runtime      => ['ros-noetic-message-runtime']
Build and Build Tool Dependencies:
  rosdep key           => focal key
  message_generation   => ['ros-noetic-message-generation']
  std_msgs             => ['ros-noetic-std-msgs']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-msgs_1.1.0-1_focal
####
#### Successfully generated 'focal' debian for package 'diffbot_msgs' at version '1.1.0-1'
####


==> git-bloom-generate -y rosdebian --prefix release/noetic noetic -i 1 --os-name debian --os-not-required
Generating source debs for the packages: ['diffbot_msgs', 'diffbot_gazebo', 'diffbot_description', 'diffbot_robot', 'diffbot_slam', 'diffbot_base', 'diffbot_control', 'diffbot_bringup', 'diffbot_mbf', 'diffbot_navigation']
Debian Incremental Version: 1
Debian Distributions: ['buster']
Releasing for rosdistro: noetic

Pre-verifying Debian dependency keys...
Running 'rosdep update'...
All keys are OK

Placing debian template files into 'debian/noetic/diffbot_msgs' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_msgs' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-msgs' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  std_msgs             => ['ros-noetic-std-msgs']
  message_runtime      => ['ros-noetic-message-runtime']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  message_generation   => ['ros-noetic-message-generation']
  std_msgs             => ['ros-noetic-std-msgs']
  catkin               => ['ros-noetic-catkin']
ROS Distro index file associate with commit '725def91e71e2e1a9520416feb916c802ed75314'
New ROS Distro index url: 'https://raw.githubusercontent.com/ros/rosdistro/725def91e71e2e1a9520416feb916c802ed75314/index-v4.yaml'
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-msgs_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_msgs' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_gazebo' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_gazebo' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-gazebo' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  gazebo_plugins       => ['ros-noetic-gazebo-plugins']
  gazebo_ros           => ['ros-noetic-gazebo-ros']
  gazebo_ros_control   => ['ros-noetic-gazebo-ros-control']
  diffbot_control      => ['ros-noetic-diffbot-control']
  diffbot_description  => ['ros-noetic-diffbot-description']
  xacro                => ['ros-noetic-xacro']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-gazebo_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_gazebo' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_description' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_description' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-description' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  joint_state_publisher => ['ros-noetic-joint-state-publisher']
  robot_state_publisher => ['ros-noetic-robot-state-publisher']
  rviz                 => ['ros-noetic-rviz']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  joint_state_publisher => ['ros-noetic-joint-state-publisher']
  robot_state_publisher => ['ros-noetic-robot-state-publisher']
  rviz                 => ['ros-noetic-rviz']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-description_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_description' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_robot' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_robot' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-robot' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  diffbot_base         => ['ros-noetic-diffbot-base']
  diffbot_bringup      => ['ros-noetic-diffbot-bringup']
  diffbot_control      => ['ros-noetic-diffbot-control']
  diffbot_description  => ['ros-noetic-diffbot-description']
  diffbot_gazebo       => ['ros-noetic-diffbot-gazebo']
  diffbot_navigation   => ['ros-noetic-diffbot-navigation']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-robot_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_robot' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_slam' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_slam' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-slam' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  gmapping             => ['ros-noetic-gmapping']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-slam_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_slam' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_base' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_base' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-base' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  diagnostic_updater   => ['ros-noetic-diagnostic-updater']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  controller_manager   => ['ros-noetic-controller-manager']
  control_toolbox      => ['ros-noetic-control-toolbox']
  dynamic_reconfigure  => ['ros-noetic-dynamic-reconfigure']
  urdf                 => ['ros-noetic-urdf']
  roscpp               => ['ros-noetic-roscpp']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  diffbot_msgs         => ['ros-noetic-diffbot-msgs']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  rosserial            => ['ros-noetic-rosserial']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  diagnostic_updater   => ['ros-noetic-diagnostic-updater']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  controller_manager   => ['ros-noetic-controller-manager']
  control_toolbox      => ['ros-noetic-control-toolbox']
  dynamic_reconfigure  => ['ros-noetic-dynamic-reconfigure']
  urdf                 => ['ros-noetic-urdf']
  roscpp               => ['ros-noetic-roscpp']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  diffbot_msgs         => ['ros-noetic-diffbot-msgs']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-base_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_base' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_control' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_control' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-control' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  controller_manager   => ['ros-noetic-controller-manager']
  joint_state_controller => ['ros-noetic-joint-state-controller']
  robot_state_publisher => ['ros-noetic-robot-state-publisher']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  roscpp               => ['ros-noetic-roscpp']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  transmission_interface => ['ros-noetic-transmission-interface']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  controller_manager   => ['ros-noetic-controller-manager']
  diff_drive_controller => ['ros-noetic-diff-drive-controller']
  hardware_interface   => ['ros-noetic-hardware-interface']
  roscpp               => ['ros-noetic-roscpp']
  rosparam_shortcuts   => ['ros-noetic-rosparam-shortcuts']
  sensor_msgs          => ['ros-noetic-sensor-msgs']
  transmission_interface => ['ros-noetic-transmission-interface']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-control_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_control' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_bringup' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_bringup' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-bringup' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  teleop_twist_keyboard => ['ros-noetic-teleop-twist-keyboard']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-bringup_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_bringup' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_mbf' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_mbf' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-mbf' has dependencies:
Build and Build Tool Dependencies:
  rosdep key           => buster key
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-mbf_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_mbf' at version '1.1.0-1'
####

Placing debian template files into 'debian/noetic/diffbot_navigation' branch.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'buster' debian for package 'diffbot_navigation' at version '1.1.0-1'
####
Generating debian for buster...
No homepage set, defaulting to ''
No historical releaser history, using current maintainer name and email for each versioned changelog entry.
Package 'diffbot-navigation' has dependencies:
Run Dependencies:
  rosdep key           => buster key
  amcl                 => ['ros-noetic-amcl']
  diffbot_bringup      => ['ros-noetic-diffbot-bringup']
  map_server           => ['ros-noetic-map-server']
  move_base            => ['ros-noetic-move-base']
  base_local_planner   => ['ros-noetic-base-local-planner']
  dwa_local_planner    => ['ros-noetic-dwa-local-planner']
Build and Build Tool Dependencies:
  rosdep key           => buster key
  amcl                 => ['ros-noetic-amcl']
  diffbot_bringup      => ['ros-noetic-diffbot-bringup']
  map_server           => ['ros-noetic-map-server']
  move_base            => ['ros-noetic-move-base']
  catkin               => ['ros-noetic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/control.em' -> 'debian/control'
Creating tag: debian/ros-noetic-diffbot-navigation_1.1.0-1_buster
####
#### Successfully generated 'buster' debian for package 'diffbot_navigation' at version '1.1.0-1'
####


==> git-bloom-generate -y rosrpm --prefix release/noetic noetic -i 1 --os-name fedora
No platforms defined for os 'fedora' in release file for the 'noetic' distro.
Not performing RPM generation.

==> git-bloom-generate -y rosrpm --prefix release/noetic noetic -i 1 --os-name rhel
No platforms defined for os 'rhel' in release file for the 'noetic' distro.
Not performing RPM generation.




Tip: Check to ensure that the debian tags created have the same version as the upstream version you are releasing.
Everything went as expected, you should check that the new tags match your expectations, and then push to the release repo with:
  git push --all && git push --tags  # You might have to add --force to the second command if you are over-writing existing tags
<== Released 'diffbot' using release track 'noetic' successfully
==> git remote -v
origin  https://github.com/ros-mobile-robots-release/diffbot-release.git (fetch)
origin  https://github.com/ros-mobile-robots-release/diffbot-release.git (push)
Releasing complete, push to release repository?
Continue [Y/n]? Y
==> Pushing changes to release repository for 'diffbot'
==> git push --all
Enumerating objects: 1108, done.
Counting objects: 100% (1108/1108), done.
Delta compression using up to 48 threads
Compressing objects: 100% (996/996), done.
Writing objects: 100% (1108/1108), 4.85 MiB | 1.61 MiB/s, done.
Total 1108 (delta 243), reused 0 (delta 0)
remote: Resolving deltas: 100% (243/243), done.
To https://github.com/ros-mobile-robots-release/diffbot-release.git
 * [new branch]      debian/noetic/buster/diffbot_base -> debian/noetic/buster/diffbot_base
 * [new branch]      debian/noetic/buster/diffbot_bringup -> debian/noetic/buster/diffbot_bringup
 * [new branch]      debian/noetic/buster/diffbot_control -> debian/noetic/buster/diffbot_control
 * [new branch]      debian/noetic/buster/diffbot_description -> debian/noetic/buster/diffbot_description
 * [new branch]      debian/noetic/buster/diffbot_gazebo -> debian/noetic/buster/diffbot_gazebo
 * [new branch]      debian/noetic/buster/diffbot_mbf -> debian/noetic/buster/diffbot_mbf
 * [new branch]      debian/noetic/buster/diffbot_msgs -> debian/noetic/buster/diffbot_msgs
 * [new branch]      debian/noetic/buster/diffbot_navigation -> debian/noetic/buster/diffbot_navigation
 * [new branch]      debian/noetic/buster/diffbot_robot -> debian/noetic/buster/diffbot_robot
 * [new branch]      debian/noetic/buster/diffbot_slam -> debian/noetic/buster/diffbot_slam
 * [new branch]      debian/noetic/diffbot_base -> debian/noetic/diffbot_base
 * [new branch]      debian/noetic/diffbot_bringup -> debian/noetic/diffbot_bringup
 * [new branch]      debian/noetic/diffbot_control -> debian/noetic/diffbot_control
 * [new branch]      debian/noetic/diffbot_description -> debian/noetic/diffbot_description
 * [new branch]      debian/noetic/diffbot_gazebo -> debian/noetic/diffbot_gazebo
 * [new branch]      debian/noetic/diffbot_mbf -> debian/noetic/diffbot_mbf
 * [new branch]      debian/noetic/diffbot_msgs -> debian/noetic/diffbot_msgs
 * [new branch]      debian/noetic/diffbot_navigation -> debian/noetic/diffbot_navigation
 * [new branch]      debian/noetic/diffbot_robot -> debian/noetic/diffbot_robot
 * [new branch]      debian/noetic/diffbot_slam -> debian/noetic/diffbot_slam
 * [new branch]      debian/noetic/focal/diffbot_base -> debian/noetic/focal/diffbot_base
 * [new branch]      debian/noetic/focal/diffbot_bringup -> debian/noetic/focal/diffbot_bringup
 * [new branch]      debian/noetic/focal/diffbot_control -> debian/noetic/focal/diffbot_control
 * [new branch]      debian/noetic/focal/diffbot_description -> debian/noetic/focal/diffbot_description
 * [new branch]      debian/noetic/focal/diffbot_gazebo -> debian/noetic/focal/diffbot_gazebo
 * [new branch]      debian/noetic/focal/diffbot_mbf -> debian/noetic/focal/diffbot_mbf
 * [new branch]      debian/noetic/focal/diffbot_msgs -> debian/noetic/focal/diffbot_msgs
 * [new branch]      debian/noetic/focal/diffbot_navigation -> debian/noetic/focal/diffbot_navigation
 * [new branch]      debian/noetic/focal/diffbot_robot -> debian/noetic/focal/diffbot_robot
 * [new branch]      debian/noetic/focal/diffbot_slam -> debian/noetic/focal/diffbot_slam
 * [new branch]      master -> master
 * [new branch]      patches/debian/noetic/buster/diffbot_base -> patches/debian/noetic/buster/diffbot_base
 * [new branch]      patches/debian/noetic/buster/diffbot_bringup -> patches/debian/noetic/buster/diffbot_bringup
 * [new branch]      patches/debian/noetic/buster/diffbot_control -> patches/debian/noetic/buster/diffbot_control
 * [new branch]      patches/debian/noetic/buster/diffbot_description -> patches/debian/noetic/buster/diffbot_description
 * [new branch]      patches/debian/noetic/buster/diffbot_gazebo -> patches/debian/noetic/buster/diffbot_gazebo
 * [new branch]      patches/debian/noetic/buster/diffbot_mbf -> patches/debian/noetic/buster/diffbot_mbf
 * [new branch]      patches/debian/noetic/buster/diffbot_msgs -> patches/debian/noetic/buster/diffbot_msgs
 * [new branch]      patches/debian/noetic/buster/diffbot_navigation -> patches/debian/noetic/buster/diffbot_navigation
 * [new branch]      patches/debian/noetic/buster/diffbot_robot -> patches/debian/noetic/buster/diffbot_robot
 * [new branch]      patches/debian/noetic/buster/diffbot_slam -> patches/debian/noetic/buster/diffbot_slam
 * [new branch]      patches/debian/noetic/diffbot_base -> patches/debian/noetic/diffbot_base
 * [new branch]      patches/debian/noetic/diffbot_bringup -> patches/debian/noetic/diffbot_bringup
 * [new branch]      patches/debian/noetic/diffbot_control -> patches/debian/noetic/diffbot_control
 * [new branch]      patches/debian/noetic/diffbot_description -> patches/debian/noetic/diffbot_description
 * [new branch]      patches/debian/noetic/diffbot_gazebo -> patches/debian/noetic/diffbot_gazebo
 * [new branch]      patches/debian/noetic/diffbot_mbf -> patches/debian/noetic/diffbot_mbf
 * [new branch]      patches/debian/noetic/diffbot_msgs -> patches/debian/noetic/diffbot_msgs
 * [new branch]      patches/debian/noetic/diffbot_navigation -> patches/debian/noetic/diffbot_navigation
 * [new branch]      patches/debian/noetic/diffbot_robot -> patches/debian/noetic/diffbot_robot
 * [new branch]      patches/debian/noetic/diffbot_slam -> patches/debian/noetic/diffbot_slam
 * [new branch]      patches/debian/noetic/focal/diffbot_base -> patches/debian/noetic/focal/diffbot_base
 * [new branch]      patches/debian/noetic/focal/diffbot_bringup -> patches/debian/noetic/focal/diffbot_bringup
 * [new branch]      patches/debian/noetic/focal/diffbot_control -> patches/debian/noetic/focal/diffbot_control
 * [new branch]      patches/debian/noetic/focal/diffbot_description -> patches/debian/noetic/focal/diffbot_description
 * [new branch]      patches/debian/noetic/focal/diffbot_gazebo -> patches/debian/noetic/focal/diffbot_gazebo
 * [new branch]      patches/debian/noetic/focal/diffbot_mbf -> patches/debian/noetic/focal/diffbot_mbf
 * [new branch]      patches/debian/noetic/focal/diffbot_msgs -> patches/debian/noetic/focal/diffbot_msgs
 * [new branch]      patches/debian/noetic/focal/diffbot_navigation -> patches/debian/noetic/focal/diffbot_navigation
 * [new branch]      patches/debian/noetic/focal/diffbot_robot -> patches/debian/noetic/focal/diffbot_robot
 * [new branch]      patches/debian/noetic/focal/diffbot_slam -> patches/debian/noetic/focal/diffbot_slam
 * [new branch]      patches/release/noetic/diffbot_base -> patches/release/noetic/diffbot_base
 * [new branch]      patches/release/noetic/diffbot_bringup -> patches/release/noetic/diffbot_bringup
 * [new branch]      patches/release/noetic/diffbot_control -> patches/release/noetic/diffbot_control
 * [new branch]      patches/release/noetic/diffbot_description -> patches/release/noetic/diffbot_description
 * [new branch]      patches/release/noetic/diffbot_gazebo -> patches/release/noetic/diffbot_gazebo
 * [new branch]      patches/release/noetic/diffbot_mbf -> patches/release/noetic/diffbot_mbf
 * [new branch]      patches/release/noetic/diffbot_msgs -> patches/release/noetic/diffbot_msgs
 * [new branch]      patches/release/noetic/diffbot_navigation -> patches/release/noetic/diffbot_navigation
 * [new branch]      patches/release/noetic/diffbot_robot -> patches/release/noetic/diffbot_robot
 * [new branch]      patches/release/noetic/diffbot_slam -> patches/release/noetic/diffbot_slam
 * [new branch]      release/noetic/diffbot_base -> release/noetic/diffbot_base
 * [new branch]      release/noetic/diffbot_bringup -> release/noetic/diffbot_bringup
 * [new branch]      release/noetic/diffbot_control -> release/noetic/diffbot_control
 * [new branch]      release/noetic/diffbot_description -> release/noetic/diffbot_description
 * [new branch]      release/noetic/diffbot_gazebo -> release/noetic/diffbot_gazebo
 * [new branch]      release/noetic/diffbot_mbf -> release/noetic/diffbot_mbf
 * [new branch]      release/noetic/diffbot_msgs -> release/noetic/diffbot_msgs
 * [new branch]      release/noetic/diffbot_navigation -> release/noetic/diffbot_navigation
 * [new branch]      release/noetic/diffbot_robot -> release/noetic/diffbot_robot
 * [new branch]      release/noetic/diffbot_slam -> release/noetic/diffbot_slam
 * [new branch]      upstream -> upstream
<== Pushed changes successfully
==> Pushing tags to release repository for 'diffbot'
==> git push --tags
Total 0 (delta 0), reused 0 (delta 0)
To https://github.com/ros-mobile-robots-release/diffbot-release.git
 * [new tag]         debian/ros-noetic-diffbot-base_1.1.0-1_buster -> debian/ros-noetic-diffbot-base_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-base_1.1.0-1_focal -> debian/ros-noetic-diffbot-base_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-bringup_1.1.0-1_buster -> debian/ros-noetic-diffbot-bringup_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-bringup_1.1.0-1_focal -> debian/ros-noetic-diffbot-bringup_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-control_1.1.0-1_buster -> debian/ros-noetic-diffbot-control_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-control_1.1.0-1_focal -> debian/ros-noetic-diffbot-control_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-description_1.1.0-1_buster -> debian/ros-noetic-diffbot-description_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-description_1.1.0-1_focal -> debian/ros-noetic-diffbot-description_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-gazebo_1.1.0-1_buster -> debian/ros-noetic-diffbot-gazebo_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-gazebo_1.1.0-1_focal -> debian/ros-noetic-diffbot-gazebo_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-mbf_1.1.0-1_buster -> debian/ros-noetic-diffbot-mbf_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-mbf_1.1.0-1_focal -> debian/ros-noetic-diffbot-mbf_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-msgs_1.1.0-1_buster -> debian/ros-noetic-diffbot-msgs_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-msgs_1.1.0-1_focal -> debian/ros-noetic-diffbot-msgs_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-navigation_1.1.0-1_buster -> debian/ros-noetic-diffbot-navigation_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-navigation_1.1.0-1_focal -> debian/ros-noetic-diffbot-navigation_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-robot_1.1.0-1_buster -> debian/ros-noetic-diffbot-robot_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-robot_1.1.0-1_focal -> debian/ros-noetic-diffbot-robot_1.1.0-1_focal
 * [new tag]         debian/ros-noetic-diffbot-slam_1.1.0-1_buster -> debian/ros-noetic-diffbot-slam_1.1.0-1_buster
 * [new tag]         debian/ros-noetic-diffbot-slam_1.1.0-1_focal -> debian/ros-noetic-diffbot-slam_1.1.0-1_focal
 * [new tag]         release/noetic/diffbot_base/1.1.0-1 -> release/noetic/diffbot_base/1.1.0-1
 * [new tag]         release/noetic/diffbot_bringup/1.1.0-1 -> release/noetic/diffbot_bringup/1.1.0-1
 * [new tag]         release/noetic/diffbot_control/1.1.0-1 -> release/noetic/diffbot_control/1.1.0-1
 * [new tag]         release/noetic/diffbot_description/1.1.0-1 -> release/noetic/diffbot_description/1.1.0-1
 * [new tag]         release/noetic/diffbot_gazebo/1.1.0-1 -> release/noetic/diffbot_gazebo/1.1.0-1
 * [new tag]         release/noetic/diffbot_mbf/1.1.0-1 -> release/noetic/diffbot_mbf/1.1.0-1
 * [new tag]         release/noetic/diffbot_msgs/1.1.0-1 -> release/noetic/diffbot_msgs/1.1.0-1
 * [new tag]         release/noetic/diffbot_navigation/1.1.0-1 -> release/noetic/diffbot_navigation/1.1.0-1
 * [new tag]         release/noetic/diffbot_robot/1.1.0-1 -> release/noetic/diffbot_robot/1.1.0-1
 * [new tag]         release/noetic/diffbot_slam/1.1.0-1 -> release/noetic/diffbot_slam/1.1.0-1
 * [new tag]         upstream/1.1.0 -> upstream/1.1.0
<== Pushed tags successfully
==> Generating pull request to distro file located at 'https://raw.githubusercontent.com/ros/rosdistro/725def91e71e2e1a9520416feb916c802ed75314/noetic/distribution.yaml'
Would you like to add documentation information for this repository? [Y/n]? Y
==> Looking for a doc entry for this repository in a different distribution...
No existing doc entries found for use as defaults.
Please enter your repository information for the doc generation job.
This information should point to the repository from which documentation should be generated.
VCS Type must be one of git, svn, hg, or bzr.
VCS type: git
VCS url: https://github.com/ros-mobile-robots/diffbot.git
VCS version must be a branch, tag, or commit, e.g. master or 0.1.0
VCS version: 1.1.0
Would you like to add source information for this repository? [Y/n]? Y
==> Looking for a source entry for this repository in a different distribution...
No existing source entries found for use as defaults.
Please enter information which points to the active development branch for this repository.
This information is used to run continuous integration jobs and for developers to checkout from.
VCS Type must be one of git, svn, hg, or bzr.
VCS type: git
VCS url: https://github.com/ros-mobile-robots/diffbot.git
VCS version must be a branch, tag, or commit, e.g. master or 0.1.0
VCS version: noetic-devel
Since you are on github we can add a job to run your tests on each pull request.If you would like to turn this on please see http://wiki.ros.org/buildfarm/Pull%20request%20testing for more information. There is more setup required to setup the hooks correctly.
Would you like to turn on pull request testing? [y/N]? N
Would you like to add a maintenance status for this repository? [Y/n]? Y
Please enter a maintenance status.
Valid maintenance statuses:
- developed: active development is in progress
- maintained: no new development, but bug fixes and pull requests are addressed
- unmaintained: looking for new maintainer, bug fixes and pull requests will not be addressed
- end-of-life: should not be used, will disappear at some point
Status: developed
You can also enter a status description.
This is usually reserved for giving a reason when a status is 'end-of-life'.
Status Description [press Enter for no change]:
Unified diff for the ROS distro file located at '/tmp/tmpgx40geam/diffbot-1.1.0-1.patch':
--- 725def91e71e2e1a9520416feb916c802ed75314/noetic/distribution.yaml
+++ 725def91e71e2e1a9520416feb916c802ed75314/noetic/distribution.yaml
@@ -1609,6 +1609,32 @@
       url: https://github.com/ros/diagnostics.git
       version: noetic-devel
     status: maintained
+  diffbot:
+    doc:
+      type: git
+      url: https://github.com/ros-mobile-robots/diffbot.git
+      version: 1.1.0
+    release:
+      packages:
+      - diffbot_base
+      - diffbot_bringup
+      - diffbot_control
+      - diffbot_description
+      - diffbot_gazebo
+      - diffbot_mbf
+      - diffbot_msgs
+      - diffbot_navigation
+      - diffbot_robot
+      - diffbot_slam
+      tags:
+        release: release/noetic/{package}/{version}
+      url: https://github.com/ros-mobile-robots-release/diffbot-release.git
+      version: 1.1.0-1
+    source:
+      type: git
+      url: https://github.com/ros-mobile-robots/diffbot.git
+      version: 1.1.0
+    status: developed
   dingo:
     doc:
       type: git
==> Checking on GitHub for a fork to make the pull request from...
==> Using this fork to make a pull request from: fjp/rosdistro
==> Cloning fjp/rosdistro...
==> mkdir -p rosdistro
==> git init
Initialized empty Git repository in /tmp/zis9xzky/rosdistro/.git/
Pull Request Title: diffbot: 1.1.0-1 in 'noetic/distribution.yaml' [bloom]
Pull Request Body :
Increasing version of package(s) in repository `diffbot` to `1.1.0-1`:

- upstream repository: https://github.com/ros-mobile-robots/diffbot.git
- release repository: https://github.com/ros-mobile-robots-release/diffbot-release.git
- distro file: `noetic/distribution.yaml`
- bloom version: `0.10.7`
- previous version for package: `null`

## diffbot_base

```
* fix: missing subscriber initialization (#54 <https://github.com/ros-mobile-robots/diffbot/issues/54>)
* feat: warn if eStop() is called on mcu
* Change base config: switch motor pins
* feat: add callbacks for low level pids on mcu #54 <https://github.com/ros-mobile-robots/diffbot/issues/54>
  For MCU firmware
  - Add subscribers for left and right motor pid controllers using
  diffbot_msgs::PIDStamped custom message
  - Update debug logging message (different formatting)
  - Update PID controller interface (provide proportional_, integral_ and
  derivative_ values)
* fix: missing diffbot namespace in test
* fix: test_encoders.cpp include name
* feat: add documentation link for unity
* Merge pull request #42 <https://github.com/ros-mobile-robots/diffbot/issues/42> from joeuser846/motor-defines
  Use MOTOR_LEFT/RIGHT from diffbot_base_config.h instead of hardcoding pin numbers
* Fix filenames
* Use MOTOR_LEFT/RIGHT instead of hardcoding pin numbers
* Contributors: Franz Pucher, Joe User
```

## diffbot_bringup

```
* fix rplidar_laser_link name issue (#40 <https://github.com/ros-mobile-robots/diffbot/issues/40>, #53 <https://github.com/ros-mobile-robots/diffbot/issues/53>)
  - Rename rplidar_gpu_laser_link to rplidar_laser_link in bringup_with_laser.launch
  - Add rplidar.launch to diffbot_bringup to support framed_id argument
  - Make use of new diffbot_bringup/launch/rplidar.launch in bringup_with_laser.launch
  This solves issues in RViz:
  Transform [sender=unknown_publisher]
  For frame [rplidar_gpu_laser_link]: Frame [rplidar_gpu_laser_link] does not exist
  and in the terminal from whic diffbot_slam is launched:
  [ WARN] [1635345613.864692611]: MessageFilter [target=odom ]: Dropped 100.00% of messages so far. Please turn the [ros.gmapping.message_filter] rosconsole logger to DEBUG for more information.
* Contributors: Franz Pucher
```

## diffbot_control

```
* fix robot spawning in world origin using pid gains (#57 <https://github.com/ros-mobile-robots/diffbot/issues/57>)
* include pid.yaml to avoid Gazebo error messages
* Contributors: Franz Pucher
```

## diffbot_description

```
* fix deprecated warning using load_yaml (#53 <https://github.com/ros-mobile-robots/diffbot/issues/53>)
  Using xacro.load_yaml instead of just load_yaml
* Contributors: Franz Pucher
```

## diffbot_gazebo

```
* Update diffbot_gazebo/diffbot_view.launch
  Use db_world as default instead of corridor world
* Contributors: Franz Pucher
```

## diffbot_mbf

- No changes

## diffbot_msgs

- No changes

## diffbot_navigation

```
* Fix minor error in amcl.launch
  The "kld_err" was initialized twice, and with the wrong value. Corrected to initialize the "kld_error" parameter to 0.01 and "kld_z" to 0.99.
* Contributors: Rodrigo Silverio
```

## diffbot_robot

- No changes

## diffbot_slam

- No changes

Open a pull request from 'fjp/rosdistro:bloom-diffbot-0' into 'ros/rosdistro:master'?
Continue [Y/n]?
==> git checkout -b bloom-diffbot-0
Switched to a new branch 'bloom-diffbot-0'
==> Pulling latest rosdistro branch
remote: Enumerating objects: 170051, done.
remote: Counting objects: 100% (86/86), done.
remote: Compressing objects: 100% (65/65), done.
remote: Total 170051 (delta 41), reused 0 (delta 0), pack-reused 169965
Receiving objects: 100% (170051/170051), 105.06 MiB | 4.12 MiB/s, done.
Resolving deltas: 100% (101945/101945), done.
From https://github.com/ros/rosdistro
 * branch            master     -> FETCH_HEAD
==> git reset --hard 725def91e71e2e1a9520416feb916c802ed75314
HEAD is now at 725def91e Retarget ament_acceleration repos to rolling branch (#32587)
==> Writing new distribution file: noetic/distribution.yaml
==> git add noetic/distribution.yaml
==> git commit -m "diffbot: 1.1.0-1 in 'noetic/distribution.yaml' [bloom]"
[bloom-diffbot-0 57ef230a8] diffbot: 1.1.0-1 in 'noetic/distribution.yaml' [bloom]
 1 file changed, 26 insertions(+)
==> Pushing changes to fork
Enumerating objects: 7, done.
Counting objects: 100% (7/7), done.
Delta compression using up to 48 threads
Compressing objects: 100% (3/3), done.
Writing objects: 100% (4/4), 574 bytes | 574.00 KiB/s, done.
Total 4 (delta 2), reused 0 (delta 0)
remote: Resolving deltas: 100% (2/2), completed with 2 local objects.
remote:
remote: Create a pull request for 'bloom-diffbot-0' on GitHub by visiting:
remote:      https://github.com/fjp/rosdistro/pull/new/bloom-diffbot-0
remote:
To https://github.com/fjp/rosdistro.git
 * [new branch]          bloom-diffbot-0 -> bloom-diffbot-0
<== Pull request opened at: https://github.com/ros/rosdistro/pull/32590
````

View the pull request at https://github.com/ros/rosdistro/pull/32590 and check that the tests (e.g. Nose test) pass.
