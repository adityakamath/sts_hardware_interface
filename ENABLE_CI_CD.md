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

#### 3. [.github/workflows/lint.yml](.github/workflows/lint.yml)

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
```

#### 4. [.github/workflows/pr-comments-check.yml](.github/workflows/pr-comments-check.yml)

Change:
```yaml
on:
  # pull_request:
  #   types: [opened, synchronize, reopened, edited]
  # pull_request_review:
  #   ...
  workflow_dispatch:
```

To:
```yaml
on:
  pull_request:
    types: [opened, synchronize, reopened, edited]
  pull_request_review:
    types: [submitted, edited, dismissed]
  pull_request_review_comment:
    types: [created, edited, deleted]
  issue_comment:
    types: [created, edited, deleted]
```

#### 5. [.github/workflows/release.yml](.github/workflows/release.yml)

Change:
```yaml
on:
  # push:
  #   tags:
  #     - 'v*'
  workflow_dispatch:
```

To:
```yaml
on:
  push:
    tags:
      - 'v*'
  workflow_dispatch:
```

### Step 3: Update Badge URLs

In [README.md](README.md), replace `YOUR_USERNAME` with your actual GitHub username in all badge URLs.

Find and replace:
- `YOUR_USERNAME` → your actual GitHub username

## Quick Enable Script

Or use this one-liner to uncomment all at once:

```bash
# Remove the comment markers from workflow files
sed -i 's/^  # /  /g' .github/workflows/*.yml
sed -i 's/^# DISABLED.*//g' .github/workflows/*.yml
```

## Verify It's Working

1. Push the enabled workflows to GitHub
2. Create a test PR
3. Check the "Actions" tab - you should see workflows running
4. Badges in README should show "passing" status

## Cost Information

| Repository Type | GitHub Actions Minutes | Your Workflows |
|----------------|----------------------|----------------|
| **Private** | 2,000 min/month (free tier) | ~25 min per PR |
| **Public** | ♾️ **UNLIMITED & FREE** | No cost! |

## What Each Workflow Does

Once enabled:

- ✅ **Build & Test** - Tests on Humble, Iron, Jazzy, Kilted, Rolling
- ✅ **clang-tidy** - Static analysis, finds bugs
- ✅ **Lint** - Code style checks
- ✅ **PR Comments Check** - Enforces review resolution
- ✅ **Release** - Automated releases on tags

See [docs/CI_CD.md](docs/CI_CD.md) for complete documentation.

---

**Current Status**: All workflows disabled to save minutes on private repo.
**Action Required**: Enable when making repository public for free unlimited CI/CD!
