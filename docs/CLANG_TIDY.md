# clang-tidy Integration

This package includes clang-tidy static analysis to improve code quality and catch potential issues.

## What is clang-tidy?

clang-tidy is a clang-based C++ "linter" tool that provides an extensible framework for diagnosing and fixing typical programming errors, style violations, and interface misuse.

## Installation

### macOS
```bash
brew install llvm
```

### Ubuntu/Debian
```bash
sudo apt install clang-tidy
```

## Usage

### Method 1: Enable During Build (Recommended)

Build the package with clang-tidy enabled:

```bash
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface --cmake-args -DENABLE_CLANG_TIDY=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

This will run clang-tidy checks during compilation and report issues in real-time.

### Method 2: Run with ament_cmake tests

```bash
cd ~/ros2_ws
colcon test --packages-select sts_hardware_interface --event-handlers console_direct+
```

### Method 3: Manual Invocation

Run clang-tidy on specific files:

```bash
clang-tidy -p build/sts_hardware_interface src/sts_hardware_interface.cpp
```

## GitHub Actions Integration

The package includes a GitHub Actions workflow that automatically runs clang-tidy on:

- All pull requests that modify C++ files
- Pushes to main/develop branches
- Manual workflow dispatch

The workflow:

1. Builds the package with ROS 2 Jazzy
2. Runs clang-tidy on all source files
3. Uploads results as artifacts
4. Comments on PRs if issues are found

See [.github/workflows/clang-tidy.yml](../.github/workflows/clang-tidy.yml) for details.

## Configuration

The `.clang-tidy` file in the package root contains the configuration for checks. The current setup:

- Enables most standard C++ checks
- Follows ROS 2 naming conventions
- Disables overly pedantic checks that may not be relevant
- Configured for C++17

### Key Checks Enabled:
- **bugprone-**: Detects suspicious constructs that are likely to be bugs
- **cert-**: CERT C++ Coding Standard checks
- **cppcoreguidelines-**: C++ Core Guidelines checks
- **modernize-**: Suggests modern C++ replacements for legacy code
- **performance-**: Performance-related checks
- **readability-**: Code readability improvements
- **clang-analyzer-**: Deep static analysis checks

### Customizing Checks

Edit `.clang-tidy` to enable/disable specific checks. Add checks to the `Checks:` section with:
- Prefix with `-` to disable: `-modernize-use-trailing-return-type`
- No prefix or `+` to enable: `modernize-use-auto`

## Integration with IDEs

### VS Code
Install the "C/C++" extension by Microsoft or "clangd" extension. Both support clang-tidy when `compile_commands.json` is available.

### CLion
Built-in support. Go to Settings → Editor → Inspections → C/C++ → General → Clang-Tidy

### Vim/Neovim
Use ALE or CoC with clangd LSP server.

## Fixing Issues Automatically

Some issues can be auto-fixed:

```bash
clang-tidy -p build/sts_hardware_interface --fix src/sts_hardware_interface.cpp
```

**Warning:** Review changes before committing, as auto-fixes may sometimes be incorrect.

## Common Issues and Solutions

### Issue: "compile_commands.json not found"
**Solution:** Build with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`:
```bash
colcon build --packages-select sts_hardware_interface --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Issue: Too many warnings
**Solution:** Adjust the checks in `.clang-tidy` or use `-checks=` flag to filter

### Issue: False positives
**Solution:** Add `// NOLINT` or `// NOLINTNEXTLINE` comments to suppress specific warnings

Example:
```cpp
// NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
auto* ptr = reinterpret_cast<uint8_t*>(data);
```

## Best Practices

1. **Run early and often**: Integrate clang-tidy into your development workflow
2. **Fix issues progressively**: Don't try to fix everything at once
3. **Understand warnings**: Learn what each check does before disabling it
4. **Keep configuration updated**: Review and update `.clang-tidy` as the project evolves
5. **Use in CI/CD**: Consider adding clang-tidy checks to your continuous integration pipeline

## References

- [clang-tidy documentation](https://clang.llvm.org/extra/clang-tidy/)
- [List of all checks](https://clang.llvm.org/extra/clang-tidy/checks/list.html)
- [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)
