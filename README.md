# 2026-COWORK-BSL

This repository was created with the global `newrepo` PowerShell function.

## One-time setup

Install GitHub CLI and log in once:

```powershell
. $PROFILE
gh auth login
```

If `gh` is not recognized in the current shell, reopen PowerShell and run:

```powershell
. $PROFILE
gh auth status
```

## Create a new repository from any folder

Move to the parent folder where you want the new project folder to be created, then run:

```powershell
. $PROFILE
newrepo REPO_NAME GITHUB_USERNAME public
```

Example:

```powershell
. $PROFILE
newrepo 2026-COWORK-BSL shjang-KAIST public
```

## What `newrepo` does

`newrepo` automatically:

- creates a new folder
- runs `git init`
- creates `README.md` if the folder is empty
- makes the first commit
- renames the branch to `main`
- creates the remote GitHub repository
- adds `origin`
- pushes `main`

## Important note

Run `newrepo` from the parent directory, not inside a folder with the same name.

Good:

```powershell
cd C:\Users\Seunghun Jang\Desktop\Ongoing
newrepo 2026-COWORK-BSL shjang-KAIST public
```

Not recommended:

```powershell
cd C:\Users\Seunghun Jang\Desktop\Ongoing\2026-COWORK-BSL
newrepo 2026-COWORK-BSL shjang-KAIST public
```

The second case creates a nested folder like:

```text
2026-COWORK-BSL\2026-COWORK-BSL
```

