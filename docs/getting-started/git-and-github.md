# Git and GitHub

<a id="git-setup"></a>

DiffBot's and Remo's code is on [GitHub](https://github.com/ros-mobile-robots), in [Git](https://git-scm.com/) repositories. This page sets up Git on your PC, in [step 2](index.md#2-get-the-code) of Getting Started. Reading and running the code only takes the first section, [Install Git and clone](#install-git-and-clone).

The robot's computer needs Git too, but only later: in [step 5](index.md#5-set-up-the-robots-computer), [Packages Setup](../packages/packages-setup.md#git-clone-diffbot-repository) installs it there and clones the code. The optional sections, [Commit and push changes](#commit-and-push-changes-optional) and [Private repositories and Git LFS](#private-repositories-and-git-lfs-optional), work the same way on your PC and on the robot.

## Install Git and clone

=== "Linux"

    On your Linux PC, Ubuntu usually has Git already, which `git --version` shows. Otherwise:

    ```console
    sudo apt install git
    ```

=== "Windows (WSL 2)"

    Install Git in the Ubuntu in WSL 2, not in Windows: open the Ubuntu terminal and check with `git --version`. Ubuntu usually has it already. Otherwise:

    ```console
    sudo apt install git
    ```

    Git for Windows isn't needed, because VS Code and the dev container use the Git in the Ubuntu.

The repositories are public, so cloning needs no GitHub account and no login:

```console
git clone https://github.com/ros-mobile-robots/diffbot.git
```

Where to clone on your PC: see [Use the Dev Container](../development/dev-container.md#usage).

## Commit and push changes (optional)

Only needed if you change code and want to send it to GitHub, or if you use a private repository. How changes get into the project, with forks and pull requests, is described in [Contributing](../development/index.md).

**Commit identity**, needed to commit: Git records a name and an email address in every commit. The name is how you want to appear as the author, not necessarily your GitHub username. Use an email address of your GitHub account, or GitHub's noreply address, so GitHub links the commits to you ([Setting your commit email address](https://docs.github.com/en/account-and-profile/setting-up-and-managing-your-personal-account-on-github/managing-email-preferences/setting-your-commit-email-address)):

```console
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

**Authentication**, needed to push, or to clone a private repository such as [Remo Insiders](../insiders/index.md#remo-stl-files): GitHub doesn't accept your account password for Git ([since August 2021](https://github.blog/security/application-security/token-authentication-requirements-for-git-operations/)). Use HTTPS or SSH:

=== "HTTPS"

    [Install GitHub CLI](https://github.com/cli/cli/blob/trunk/docs/install_linux.md) and sign in:

    ```console
    gh auth login
    ```

    Choose GitHub.com and HTTPS, let it authenticate Git, and log in with a web browser. Git then uses the stored token when you push. [Git Credential Manager](https://github.com/git-ecosystem/git-credential-manager) works too; GitHub describes both in [Caching your GitHub credentials in Git](https://docs.github.com/en/get-started/git-basics/caching-your-github-credentials-in-git).

=== "SSH"

    Create a key protected by a passphrase, and add the public key to your GitHub account under Settings → SSH and GPG keys:

    ```console
    ssh-keygen -t ed25519 -C "you@example.com"
    cat ~/.ssh/id_ed25519.pub
    ```

    `gh auth login` with SSH as the protocol does both steps for you. GitHub's guide: [Connecting to GitHub with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

    Then use the SSH address of a repository you can push to: your fork (see [Contributing](../development/index.md)), or a private repository you have access to. Clone with it, or switch an existing clone to it:

    ```console
    git remote set-url origin git@github.com:YOUR-ACCOUNT/diffbot.git
    ```

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
