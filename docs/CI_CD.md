# CI/CD Documentation

This package uses GitHub Actions for lightweight continuous integration and deployment suitable for an open-source DIY project.

> **⚠️ IMPORTANT**: All workflows are currently **DISABLED** to avoid costs on private repositories.
> See [ENABLE_CI_CD.md](../ENABLE_CI_CD.md) for instructions to enable them when you make the repo public (FREE unlimited minutes!).

## Overview

The CI/CD pipeline consists of five workflows:

1. **Build & Test** - Ensures code compiles and tests pass across all ROS 2 versions
2. **clang-tidy** - Static analysis for code quality
3. **Lint** - Code style and formatting checks
4. **PR Comments Check** - Enforces resolution of all review comments
5. **Release** - Automated release creation

## Workflows

### 1. Build & Test ([build-test.yml](../.github/workflows/build-test.yml))

**When it runs:**
- On every pull request
- On pushes to `main` or `develop` branches
- Manually via workflow dispatch

**What it does:**
- Builds the package on **ALL** ROS 2 distributions:
  - Humble (LTS) on Ubuntu 22.04
  - Iron on Ubuntu 22.04
  - Jazzy on Ubuntu 24.04
  - Kilted on Ubuntu 24.04
  - Rolling (latest) on Ubuntu 24.04
- Runs all tests on each distribution
- Uploads build logs and test results

**Status Badge:**
```markdown
[![Build & Test](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml)
```

### 2. clang-tidy ([clang-tidy.yml](../.github/workflows/clang-tidy.yml))

**When it runs:**
- On pull requests that change C++ files
- On pushes to `main`/`develop` that change C++ files
- Manually

**What it does:**
- Runs clang-tidy static analysis
- Checks for bugs, performance issues, style violations
- Comments on PRs with findings
- Uploads detailed results as artifacts

See [CLANG_TIDY.md](CLANG_TIDY.md) for more details.

### 3. Lint ([lint.yml](../.github/workflows/lint.yml))

**When it runs:**
- On every pull request
- On pushes to `main` or `develop` branches

**What it does:**
- Runs ROS 2 ament linters (cppcheck, cpplint, xmllint)
- Checks markdown formatting
- Validates package.xml and CMakeLists.txt

**Configured checks:**
- C++ code style (cpplint)
- Static analysis (cppcheck)
- XML formatting (package.xml, launch files)
- Markdown style (documentation)

### 4. PR Comments Check ([pr-comments-check.yml](../.github/workflows/pr-comments-check.yml))

**When it runs:**
- On every pull request event (opened, updated, edited)
- On pull request review events
- On review comment events

**What it does:**
- Checks for unresolved review conversations
- Tracks comments from:
  - CodeRabbit AI reviews
  - GitHub Copilot reviews
  - clang-tidy findings
  - Human reviewers
- **Blocks merging** if unresolved comments exist
- Updates PR with comment resolution status
- Distinguishes between bot and human comments

**Status Badge:**
```markdown
[![PR Comments Check](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/pr-comments-check.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/pr-comments-check.yml)
```

**Why this is important:**
This workflow ensures that:
- All review feedback is addressed
- No issues are accidentally ignored
- Code quality standards are maintained
- Bot suggestions (CodeRabbit, Copilot, clang-tidy) are reviewed

### 5. Release ([release.yml](../.github/workflows/release.yml))

**When it runs:**
- When a tag matching `v*` is pushed (e.g., `v0.3.1`)
- Manually via workflow dispatch

**What it does:**
- Creates GitHub release with changelog
- Builds release artifacts
- Uploads compiled binaries as tarball

## Using the CI/CD Pipeline

### Running Checks Locally

Before pushing, you can run checks locally:

```bash
# Build and test
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
colcon test --packages-select sts_hardware_interface

# Run clang-tidy
colcon build --packages-select sts_hardware_interface \
  --cmake-args -DENABLE_CLANG_TIDY=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Run linters
source /opt/ros/jazzy/setup.bash
ament_cppcheck src/sts_hardware_interface
ament_cpplint src/sts_hardware_interface
```

### Creating a Release

Two methods:

**Method 1: Git tag (recommended)**
```bash
git tag -a v0.3.1 -m "Release v0.3.1"
git push origin v0.3.1
```

**Method 2: Manual workflow**
1. Go to Actions tab on GitHub
2. Select "Release" workflow
3. Click "Run workflow"
4. Enter version number

### Pull Request Workflow

When you create a PR, the following happens automatically:

1. **Build & Test** runs to verify compilation and tests
2. **clang-tidy** analyzes code quality
3. **Lint** checks code style

All must pass before merging (you can configure this in branch protection rules).

## Adding Status Badges to README

Add these badges to your README.md:

```markdown
[![Build & Test](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml)
[![clang-tidy](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml)
[![Lint](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/lint.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/lint.yml)
```

Replace `YOUR_USERNAME` with your GitHub username.

## Branch Protection (Optional)

To enforce CI checks before merging, configure branch protection:

1. Go to repository Settings → Branches
2. Add rule for `main` branch
3. Enable "Require status checks to pass before merging"
4. Select: `Build & Test`, `clang-tidy`, `Lint`

## Customizing Workflows

### Changing ROS 2 Distribution

Edit the workflow files and change `jazzy` to your desired distro (e.g., `humble`, `rolling`).

### Adding More Platforms

To test on multiple ROS 2 versions, add a matrix to `build-test.yml`:

```yaml
strategy:
  matrix:
    ros_distro: [humble, jazzy, rolling]
```

### Disabling Specific Workflows

Comment out or delete workflows you don't need. The minimal setup would be just `build-test.yml`.

## Cost and Resource Usage

All workflows run on GitHub-hosted runners which are:
- **Free** for public repositories
- Run on Ubuntu VMs
- Have 2-core CPU, 7GB RAM
- Typical run time: 5-10 minutes per workflow

For private repos, GitHub provides 2,000 free minutes/month.

## Troubleshooting

### Build fails in CI but works locally

- Check ROS 2 version matches (Jazzy on Ubuntu 24.04)
- Verify all dependencies are in `package.xml`
- Check submodules are properly initialized

### clang-tidy finds too many issues

Adjust `.clang-tidy` configuration to disable specific checks. See [CLANG_TIDY.md](CLANG_TIDY.md).

### Tests fail intermittently

- May be timing issues in tests
- Check for race conditions
- Consider increasing timeouts

## Future Enhancements

Optional additions for growing projects:

- **Code coverage** - Track test coverage percentage
- **Documentation generation** - Auto-build Doxygen docs
- **Docker images** - Build and publish Docker images
- **Multiple platforms** - Test on macOS, Windows
- **Dependency scanning** - Automated security updates

## References

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [ROS 2 CI/CD Best Practices](https://docs.ros.org/en/rolling/How-To-Guides/Developing-a-ROS-2-Package.html)
- [ros-tooling/setup-ros](https://github.com/ros-tooling/setup-ros)
