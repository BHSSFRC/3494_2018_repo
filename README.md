3494\_2018\_repo
---

:coffee: :robot:

FRC Team 3494's 2018 source code repository.

## Getting started
1. [Set up your tools and account.](https://bhssfrc.github.io/new_prog_doc/)
2. Run the proper gradle task for your IDE.
    * `gradlew eclipse` for Eclipse
    * `gradlew idea` for IntelliJ
 
## Building, running, and deploying new code

Build: `gradlew build`

Deploy: `gradlew deploy`

You can also add `--offline` to either of those if you have no internet access (and have run any gradle command in the past with internet access.)

## Developer Workflow

### Advanced

We are using [git-flow](http://nvie.com/posts/a-successful-git-branching-model/), do that.

### Beginner

To familiarize yourself with `git`, reference the tutorial Caleb provided. Below is the most likely command workflow, but you should be familiar with git so you can handle different situations. If you are using a GUI the names on the buttons should be similar to the commands below. 

#### Starting a New Task (Feature)

1. `git checkout develop`
1. `git pull`
1. `git checkout -b <Name of feature>`

#### Adding Code Changes to the Repositories

1. `git status`
    1. Look to make sure the files changed are what you expect. You can use `git diff` if you want more information (and `q<Enter>` to quit).
1. `git add --all`
    1. This will add all the files (including new files) to the staging area to be committed. You can add individual files instead with `git add <Path to File>`
1. `git commit -m "<Message Describing Changes>"`
    1. If you need to add a long/multiline commit message you can use `git commit`. Be warned, this by default will open vim, so if you don't know how to get out you may be stuck. (`<ESC>:q<Enter>` will get you out in most cases.)
1. `git push` (or `git push -u origin HEAD` if it is the first time pushing your branch)

#### Wrapping Up Your Task

If it is a large feature you may want to create a PR through GitHub so that Caleb can review it. If it is a small feature, you can use the following to merge to develop.

1. `git checkout develop`
1. `git pull`
1. `git merge --no-ff <Name of Feature Branch>`
1. `git push`
1. Cleanup by deleting the local and origin branch
    1. `git push origin --delete <Name of Feature Branch>`
    1. `git branch --delete <Name of Feature Branch>`

## Resources

A fun git tutorial / game can be found [here.](https://learngitbranching.js.org/)
