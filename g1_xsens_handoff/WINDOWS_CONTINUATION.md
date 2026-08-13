# Continue the Project on Windows Without Losing Context

GitHub transfers the repository and durable context. Signing into the same Codex
account may expose account-backed threads, but this workflow does not depend on
chat history being synchronized.

## 1. One-time GitHub setup

Create an **empty private GitHub repository** named `G1_Xsens_Teleop_Project`
(do not initialize it with a README). The GitHub CLI (`gh`) is optional; ordinary
Git is sufficient to connect and push this project.

First inspect the local repository instead of assuming its branch or remote state:

```bash
git status
git branch --show-current
git remote -v
```

If `origin` is not listed, add it. If `origin` already exists, update it instead:

```bash
git remote add origin https://github.com/YOUR_GITHUB_USER/G1_Xsens_Teleop_Project.git
# OR, only when origin already exists:
git remote set-url origin https://github.com/YOUR_GITHUB_USER/G1_Xsens_Teleop_Project.git
```

Push the branch that actually exists locally. `HEAD` avoids hard-coding a branch
name such as `work`, `main`, or `master`:

```bash
git push -u origin HEAD
```

GitHub will request authentication during the push. With an HTTPS remote, use a
browser/credential-manager flow or a GitHub personal access token rather than an
account password. Do not paste a token into a command, chat, or repository file.

### Fix the three common first-push errors

- `gh: command not found`: do not install `gh` just for this workflow; it is not
  needed for `git push`.
- `remote origin already exists`: run `git remote -v`, then use `git remote set-url`
  if the existing URL is wrong. Do not run `git remote add origin` again.
- `src refspec work does not match any`: there is no local branch named `work`, or
  the repository has no commit yet. Check `git branch --show-current` and
  `git log -1 --oneline`, commit the intended files if necessary, then push `HEAD`.

For the repository URL `https://github.com/TroyKaravias/G1_Xsens_Teleop_Project.git`,
the safe recovery sequence is:

```bash
cd ~/Desktop/G1_Xsens_Teleop_Project_2026-08-12
git status
git branch --show-current
git log -1 --oneline
git remote -v
git remote set-url origin https://github.com/TroyKaravias/G1_Xsens_Teleop_Project.git
git push -u origin HEAD
```

Confirm on GitHub that the repository is private and that `AGENTS.md` and
`g1_xsens_handoff/AI_HANDOFF.md` are visible. Do not upload `.env` files, SSH keys,
tokens, passwords, or the local `jetson_backups/` archive.

## 2. Install the Windows tools

Install:

- Git for Windows;
- VS Code;
- the official Codex VS Code extension, signed into the same ChatGPT account;
- Xsens MVN; and
- WSL2/Ubuntu when Linux execution or simulation is needed (see `WSL2_SETUP.md`).

## 3. Clone on Windows

For editing directly in Windows PowerShell:

```powershell
cd $HOME\source
git clone https://github.com/YOUR_GITHUB_USER/G1_Xsens_Teleop_Project.git
cd G1_Xsens_Teleop_Project
git branch --show-current
code .
```

Alternatively, clone inside WSL2 and open it with VS Code's WSL integration. That
usually gives Python and shell behavior closer to the Linux development machine:

```bash
mkdir -p ~/src && cd ~/src
git clone https://github.com/YOUR_GITHUB_USER/G1_Xsens_Teleop_Project.git
cd G1_Xsens_Teleop_Project
git branch --show-current
code .
```

## 4. Restore AI context

Open the repository root, start Codex, and paste:

> Read `AGENTS.md` and `g1_xsens_handoff/AI_HANDOFF.md`, then inspect the current
> branch and recent Git history. Summarize the verified state, safety boundary,
> and next task before changing code. Do not treat tests as hardware validation.

The root `AGENTS.md` also directs future Codex agents to the handoff automatically.

## 5. Normal cross-machine routine

Before leaving either machine:

```bash
git status
git add <files-you-intend-to-save>
git commit -m "Describe the completed work"
git push
```

On the other machine, before editing:

```bash
git status
git pull --ff-only
```

Do not work on both machines with uncommitted changes at the same time. Update
`AI_HANDOFF.md` in the same commit whenever the verified state, deployment process,
safety boundary, or immediate next task changes.

## 6. Optional: keep execution on Linux

If the Linux machine remains reachable, VS Code Remote SSH can open its existing
workspace from Windows. This is useful when Windows must run MVN but Linux should
retain the Python/runtime environment. GitHub remains the backup and handoff layer
even when Remote SSH is used.
