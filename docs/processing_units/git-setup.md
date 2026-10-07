## git setup

Install git on Ubuntu via the following command:

```console
fjp@ubuntu:~$ sudo apt install git
```


Set your username and email address that you use on github (when using github to host your repository):

```console
$ git config --global user.name "github_username"
$ git config --global user.email "your.name@provider.com"
```

To store your password credentials when pushing and pulling to the remote repository use the following commands:

```console
fjp@ubuntu:~/git/2wd-robot$ git config --global credential.helper store
fjp@ubuntu:~/git/2wd-robot$ git push
Username for 'https://github.com': fjp
Password for 'https://fjp@github.com': 
Everything up-to-date
fjp@ubuntu:~/git/2wd-robot$ git push
Everything up-to-date
```
