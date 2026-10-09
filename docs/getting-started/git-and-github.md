# Git and GitHub

<a id="git-setup"></a>

DiffBot's and Remo's code is on [GitHub](https://github.com/ros-mobile-robots), in [Git](https://git-scm.com/) repositories. Both your PC and the robot need Git to get the code. Reading and running the code only takes the first section; the others are optional.

## Install Git and clone

Install Git on your Linux, on the robot, or on Windows in the Ubuntu in WSL 2. Ubuntu usually has it already, which `git --version` shows. Otherwise:

```console
sudo apt install git
```

On Windows, use this Git in the Ubuntu. Git for Windows isn't needed for the dev container.

The repositories are public, so cloning needs no GitHub account and no login:

```console
git clone https://github.com/ros-mobile-robots/diffbot.git
```

Where to clone depends on the machine. On your PC, see [Use the Dev Container](../development/dev-container.md#usage). On the robot, see [Packages Setup](../packages/packages-setup.md).

## Commit and push changes (optional)

Only needed if you change code and want to send it to GitHub, or if you use a private repository. How changes get into the project, with forks and pull requests, is described in [Contributing](../development/index.md).

**Commit identity**, needed to commit: Git records a name and an email address in every commit. The name is how you want to appear as the author, not necessarily your GitHub username. Use an email address of your GitHub account, or GitHub's noreply address, so GitHub links the commits to you ([Setting your commit email address](https://docs.github.com/en/account-and-profile/setting-up-and-managing-your-personal-account-on-github/managing-email-preferences/setting-your-commit-email-address)):

```console
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

**Authentication**, needed to push, or to clone a private repository such as [Remo Insiders](../insiders/index.md#remo-stl-files): GitHub doesn't accept your account password for Git ([since August 2021](https://github.blog/security/application-security/token-authentication-requirements-for-git-operations/)). Use one of these:

- **HTTPS** with [GitHub CLI](https://cli.github.com/) (`gh auth login`) or Git Credential Manager, as GitHub recommends in [Caching your GitHub credentials in Git](https://docs.github.com/en/get-started/git-basics/caching-your-github-credentials-in-git).
- **SSH** with a key protected by a passphrase ([Connecting to GitHub with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)).

Set this up where Git runs: on Windows, in the Ubuntu in WSL 2. In the dev container, VS Code passes your Git credentials into the container ([Sharing Git credentials](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials)).

## Private repositories and Git LFS (optional)

[Git LFS](https://git-lfs.com/) (Large File Storage) keeps large files outside the normal Git history. You need it for:

- **Remo's STL files in Remo Insiders**, a private repository, which also needs authentication (see above). How to get access: [Remo STL files](../insiders/index.md#remo-stl-files).
- **Two SVG drawings in this documentation's repository**, if you work on the docs.

Install it, and set it up once for your user:

```console
sudo apt install git-lfs
git lfs install
```

After that, `git clone` and `git pull` download the large files automatically. In a repository you cloned before installing Git LFS, run `git lfs pull`.
