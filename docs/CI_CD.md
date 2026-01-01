# CI/CD Documentation

This package uses GitHub Actions for lightweight continuous integration suitable for a DIY/maker/academic project.

> **⚠️ IMPORTANT**: All workflows are currently **DISABLED** to avoid costs on private repositories.
> See [ENABLE_CI_CD.md](../ENABLE_CI_CD.md) for instructions to enable them when you make the repo public (FREE unlimited minutes!).

## Overview

The CI/CD pipeline consists of two automated workflows plus Dependabot:

1. **Build & Test** - Ensures code compiles and tests pass across ROS 2 versions
2. **clang-tidy** - Static analysis for code quality and security
3. **Dependabot** - Automatic dependency updates (GitHub Actions, submodules)

## Workflows

### 1. Build & Test

**When it runs:**
- On every pull request
- On pushes to `main` or `develop` branches
- Manually via workflow dispatch

**What it does:**
- Builds the package on multiple ROS 2 distributions:
  - Humble (LTS) on Ubuntu 22.04
  - Iron on Ubuntu 22.04
  - Jazzy on Ubuntu 24.04
  - Kilted on Ubuntu 24.04
  - Rolling (latest) on Ubuntu 24.04
- Runs all tests on each distribution
- Uploads build logs and test results

### 2. clang-tidy

**When it runs:**
- On pull requests that change C++ files
- On pushes to `main`/`develop` that change C++ files
- Manually via workflow dispatch

**What it does:**
- Runs clang-tidy static analysis
- Checks for bugs, performance issues, style violations
- Comments on PRs with findings
- Uploads detailed results as artifacts

### 3. Dependabot

**What it does:**
- Automatically creates PRs to update GitHub Actions versions (monthly)
- Automatically creates PRs to update git submodules like SCServo_Linux (monthly)
- Keeps dependencies secure and up-to-date with zero manual effort

**Configuration**: `.github/dependabot.yml`

## clang-tidy Static Analysis

### What is clang-tidy?

clang-tidy is a clang-based C++ "linter" tool that provides an extensible framework for diagnosing and fixing typical programming errors, style violations, and interface misuse.

### Installation

**macOS**:
```bash
brew install llvm
```

**Ubuntu/Debian**:
```bash
sudo apt install clang-tidy
```

### Running Locally

**Method 1: Enable during build (recommended)**:

```bash
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface \
  --cmake-args -DENABLE_CLANG_TIDY=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

**Method 2: Run with ament tests**:

```bash
cd ~/ros2_ws
colcon test --packages-select sts_hardware_interface --event-handlers console_direct+
```

**Method 3: Manual invocation**:

```bash
clang-tidy -p build/sts_hardware_interface src/sts_hardware_interface.cpp
```

### Configuration

The `.clang-tidy` file in the package root contains the configuration for checks. Current setup:

- Enables most standard C++ checks
- Follows ROS 2 naming conventions
- Disables overly pedantic checks
- Configured for C++17

**Key checks enabled:**
- **bugprone-**: Detects suspicious constructs likely to be bugs
- **cert-**: CERT C++ Coding Standard checks
- **cppcoreguidelines-**: C++ Core Guidelines checks
- **modernize-**: Suggests modern C++ replacements
- **performance-**: Performance-related checks
- **readability-**: Code readability improvements
- **clang-analyzer-**: Deep static analysis

### Customizing Checks

Edit `.clang-tidy` to enable/disable specific checks:
- Prefix with `-` to disable: `-modernize-use-trailing-return-type`
- No prefix or `+` to enable: `modernize-use-auto`

### Auto-fixing Issues

Some issues can be auto-fixed:

```bash
clang-tidy -p build/sts_hardware_interface --fix src/sts_hardware_interface.cpp
```

**Warning:** Review changes before committing, as auto-fixes may sometimes be incorrect.

### Suppressing Warnings

Add `// NOLINT` or `// NOLINTNEXTLINE` comments:

```cpp
// NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
auto* ptr = reinterpret_cast<uint8_t*>(data);
```

### IDE Integration

**VS Code**: Install "C/C++" or "clangd" extension

**CLion**: Built-in support in Settings → Editor → Inspections → C/C++ → General → Clang-Tidy

**Vim/Neovim**: Use ALE or CoC with clangd LSP server

## Using the CI/CD Pipeline

### Running Checks Locally

Before pushing, run checks locally:

```bash
# Build and test
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
colcon test --packages-select sts_hardware_interface

# Run clang-tidy
colcon build --packages-select sts_hardware_interface \
  --cmake-args -DENABLE_CLANG_TIDY=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Pull Request Workflow

When you create a PR, the following happens automatically:

1. **Build & Test** runs to verify compilation and tests
2. **clang-tidy** analyzes code quality

Both must pass before merging (configure this in branch protection rules).

### Dependabot Updates

Dependabot automatically creates PRs when:
- GitHub Actions have new versions available
- Git submodules (SCServo_Linux) have updates

Simply review and merge these PRs to stay up-to-date.

## Adding Status Badges to README

Add these badges to your README.md:

```markdown
[![Build & Test](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml)
[![clang-tidy](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml)
```

Replace `YOUR_USERNAME` with your GitHub username.

## Branch Protection (Optional)

To enforce CI checks before merging:

1. Go to repository Settings → Branches
2. Add rule for `main` branch
3. Enable "Require status checks to pass before merging"
4. Select: `Build & Test`, `clang-tidy`

## Customizing Workflows

### Changing ROS 2 Distribution

Edit workflow files and change `jazzy` to your desired distro (e.g., `humble`, `rolling`).

### Reducing Tested Distributions

For faster CI, test only LTS and Rolling:

Edit `build-test.yml` matrix to include only:
```yaml
matrix:
  include:
    - ros_distro: humble
      os: ubuntu-22.04
    - ros_distro: rolling
      os: ubuntu-24.04
```

## Cost and Resource Usage

All workflows run on GitHub-hosted runners which are:
- **Free** for public repositories (unlimited minutes)
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

Adjust `.clang-tidy` configuration to disable specific checks.

### clang-tidy: "compile_commands.json not found"

Build with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`:
```bash
colcon build --packages-select sts_hardware_interface --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Dependabot PRs failing

- Review the changelog of the updated dependency
- Test locally before merging
- Some updates may require code changes

## Best Practices

1. **Run early and often**: Integrate clang-tidy into your development workflow
2. **Keep dependencies updated**: Merge Dependabot PRs regularly
3. **Fix issues progressively**: Don't try to fix everything at once
4. **Understand warnings**: Learn what each check does before disabling it
5. **Test locally first**: Don't rely solely on CI for catching issues

## References

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Dependabot Documentation](https://docs.github.com/en/code-security/dependabot)
- [ROS 2 CI/CD Best Practices](https://docs.ros.org/en/rolling/How-To-Guides/Developing-a-ROS-2-Package.html)
- [ros-tooling/setup-ros](https://github.com/ros-tooling/setup-ros)
- [clang-tidy documentation](https://clang.llvm.org/extra/clang-tidy/)
- [List of all clang-tidy checks](https://clang.llvm.org/extra/clang-tidy/checks/list.html)
- [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)
