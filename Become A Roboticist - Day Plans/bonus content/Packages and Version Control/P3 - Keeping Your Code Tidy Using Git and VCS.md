
**NOTE:** This video was made using an older version of the course, you can find the repo this video talks about, including the workspace.repos file here: 
https://github.com/johnny555/4robots_ws/

# What Is Git?

Git is the industry best practice way to save your code. It is a type of version control, that tracks changes to your code and allows you to roll back to previous versions easily. If you've ever saved different versions of code in folders and then struggled to find something that was different between them, then Git will help you!

# Git Terms

There are alot of terms in the Git ecosystem:

**repository (abbv. repo):** A directory containing code that is version controlled by git.

**clone (verb):** A copy of another git repo, usually local to your machine.

**commit (verb):** A small change to (one or more) file(s) in the repo.

**pull (verb):** Taking changes (commits) from a remote repo and adding it to your local copy.

**push(verb):** Sending changes to the remote repo.

**staged(noun):** Changes that are ready for a commit

but have no message.

# Handy Git Commands

Git can be pretty complex, but here are some commands that I use frequently and what they will do:

Create a copy of a repo in your computer: `git clone <repo url>`

Add a file to a commit: `git add <file_name>`

Find out what files are ready for committing (staged) and what aren’t in the commit yet: `git status`

Create a new commit: `git commit -m "commit message"`

Create a new commit and add all uncommitted files: `git commit -a -m "commit message"`

Get differences in files since last commit: `git diff`

Get changes from remote repo: `git pull`

Push your changes to remote repo: `git push`

# Branches

Sometimes you will want to experiment with an idea, but without breaking anything. You can use branches to do this. A branch will create a copy of the current code base which you can change, and then easily save and switch back to the original branch if you want to. Here are some helpful branch commands:

List branch's: `git branch`

Switch to and create a new branch: `git checkout -b <branch_name>`

Switch to an existing branch: `git checkout <branch_name>`

# Merge

When you want to bring your changes back into the original branch you will need to merge. Also if you are working with someone and they add changes you will need to merge them into your local repo. Merging can be difficult, but if you are working by yourself, usually running `git pull` will work to merge changes that you may have put on your remote branch.

If you get stuck with merging, you might need to go even deeper into git, the atlassion git tutorial is a great help: [https://www.atlassian.com/git](https://www.atlassian.com/git)

# Stash

One of my favourite git commands is stash. It will save all unstaged changes to a "stash" and then return your git to the state it was when you last committed something. This is a great way to check that your most recent changes are the thing that is broken, and not some other system issue.

Save all tracked but uncommitted changes to the stash: `git stash`

Restore the most recently stashed changes: `git stash pop`

# Git and VS Code

VS Code has inbuilt git support, which you can use if you want.

# How To Get Software From GitHub

You can click on the green "Code" button at any git repo and copy the repo url and put it into the command `git clone <copied_url>`

# ROS Workspaces

The `4robots_ws` repo that we cloned when setting up this course is a ros workspace. In side the `src` folder is where all the source for the projects we are currently working on should live, this includes any repos that we needed to get from GitHub. It can get abit tricky to manage all these different repos that are also inside of another repo, so we have a helpful tool for this.

# VCS Tool

**NOTE:** This video was made using an older version of the course, you can find the repo this video talks about, including the workspace.repos file here: 
https://github.com/johnny555/4robots_ws/

Start by looking inside the file `workspace/src/workspace.repos`:

```
repositories:
  krytn:
    type: git
    url: git@github.com:johnny555/krytn.git
    version: humble
  maci:
    type: git
    url: git@github.com:johnny555/maci.git
    version: humble
  maci_moveit:
    type: git
    url: git@github.com:johnny555/maci_moveit.git
    version: humble
  robotiq_arg85_description:
    type: git
    url: git@github.com:johnny555/robotiq_arg85_description.git
    version: humble
```

You can see that this is a list of some of the repos that are in the src folder. If you've added a new repo (such as pymoveit2), it won't show up here unless you explicitly add it.

But we can use the vcs tool to manage our `src` folder full of git repos. Here is a quick list of the top commands for vcs. (Assuming you are running it from the `/workspace` directory).

Pull all repos: `vcs pull src`

Get status of all repos: `vcs status src`

Read a repos file and clone all the repos described in that file into the `src` folder: `vcs import src < src/workspace.repos`

Create a new repo file based on the current repos in the `src` folder: `vcs export src > src/workspace.repos`

