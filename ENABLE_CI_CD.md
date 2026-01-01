# How to Enable CI/CD Workflows

## Current Status: DISABLED ⏸️

All GitHub Actions workflows are **currently disabled** because this is a private repository. Running them would consume GitHub Actions minutes (2,000 free minutes/month for private repos).

## When to Enable

Enable the workflows when you **make this repository public**. Public repositories get **unlimited free GitHub Actions minutes**! 🎉

## How to Enable (3 Steps)

### Step 1: Make Repository Public

1. Go to repository Settings on GitHub
2. Scroll to "Danger Zone"
3. Click "Change visibility"
4. Select "Make public"
5. Confirm

### Step 2: Enable Workflows

Uncomment the trigger lines in each workflow file:

#### 1. [.github/workflows/build-test.yml](.github/workflows/build-test.yml)

Change:
```yaml
on:
  # pull_request:
  # push:
  #   branches: [main, develop]
  workflow_dispatch:
```

To:
```yaml
on:
  pull_request:
  push:
    branches: [main, develop]
  workflow_dispatch:
```

#### 2. [.github/workflows/clang-tidy.yml](.github/workflows/clang-tidy.yml)

Change:
```yaml
on:
  # pull_request:
  #   paths:
  #     - '**.cpp'
  #     ...
  workflow_dispatch:
```

To:
```yaml
on:
  pull_request:
    paths:
      - '**.cpp'
      - '**.hpp'
      - '**.h'
      - 'CMakeLists.txt'
      - '.clang-tidy'
  push:
    branches:
      - main
      - develop
    paths:
      - '**.cpp'
      - '**.hpp'
      - '**.h'
      - 'CMakeLists.txt'
      - '.clang-tidy'
  workflow_dispatch:
```

### Step 3: Dependabot (Already Configured ✓)

Dependabot is already configured in [.github/dependabot.yml](.github/dependabot.yml) and will automatically:

- Check for GitHub Actions updates monthly
- Check for git submodule (SCServo_Linux) updates monthly
- Create PRs for dependency updates

No additional setup needed - it works on both private and public repos.

### Step 4: Update Badge URLs (Optional)

In [README.md](README.md), you can add status badges:

```markdown
[![Build & Test](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml)
[![clang-tidy](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml)
```

Replace `YOUR_USERNAME` with your actual GitHub username.

## Quick Enable Script

Or use this one-liner to uncomment all at once:

```bash
# Remove the comment markers from workflow trigger sections
sed -i 's/^  # /  /g' .github/workflows/build-test.yml
sed -i 's/^  # /  /g' .github/workflows/clang-tidy.yml
```

## Verify It's Working

1. Push the enabled workflows to GitHub
2. Create a test PR
3. Check the "Actions" tab - you should see workflows running
4. Badges in README should show "passing" status
5. Dependabot will start creating PRs for updates (check "Pull requests" tab)

## Cost Information

| Repository Type | GitHub Actions Minutes | Your Workflows |
|----------------|----------------------|----------------|
| **Private** | 2,000 min/month (free tier) | ~10-15 min per PR |
| **Public** | ♾️ **UNLIMITED & FREE** | No cost! |

## What Each Workflow Does

Once enabled:

- ✅ **Build & Test** - Tests compilation on Humble, Iron, Jazzy, Kilted, Rolling (~7-10 min)
- ✅ **clang-tidy** - Static analysis, finds bugs and style issues (~3-5 min)
- ✅ **Dependabot** - Automatic dependency updates (already enabled, no minutes used)

See [docs/CI_CD.md](docs/CI_CD.md) for complete documentation.

---

**Current Status**: All workflows disabled to save minutes on private repo.
**Action Required**: Enable when making repository public for free unlimited CI/CD!
