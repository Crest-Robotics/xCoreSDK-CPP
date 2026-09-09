# Prebuilt Libraries

> **Note:** This document was translated to English by Claude Code (the original Chinese/English bilingual text has been consolidated into English only).

This repository does **not** include prebuilt library files. Libraries are distributed per platform via GitHub Releases — use the Release page matching your SDK version.

**Release URL (version number is in the root `CMakeLists.txt`):**

`https://github.com/Crest-Robotics/xCoreSDK-CPP/releases/tag/v{VERSION}`

For example, the current version v0.7.1: [Release v0.7.1](https://github.com/Crest-Robotics/xCoreSDK-CPP/releases/tag/v0.7.1)

## How to Obtain

1. Clone this repository
2. Open the [Release page](https://github.com/Crest-Robotics/xCoreSDK-CPP/releases/tag/v0.7.1) matching your SDK version (see the URL pattern above; if the library is missing, running `cmake` will also print a direct link for the matching version)
3. Download the package for your platform
4. Extract it at the **repository root** so files land under `lib/`

### Package Names

| Package | Use case |
|---|---|
| `xCoreSDK-{version}-win64-release.zip` | Windows 64-bit Release build |
| `xCoreSDK-{version}-win64-debug.zip` | Windows 64-bit Debug build (includes pdb) |
| `xCoreSDK-{version}-win32-release.zip` | Windows 32-bit Release build |
| `xCoreSDK-{version}-win32-debug.zip` | Windows 32-bit Debug build (includes pdb) |
| `xCoreSDK-{version}-linux-x86_64.tar.gz` | Linux x86_64 |
| `xCoreSDK-{version}-linux-aarch64.tar.gz` | Linux aarch64 |

> For Release builds, download only the `-release` package. Download `-debug` when you need Debug symbols.

## Expected Layout After Extraction

### Windows

```
lib/Windows/Release/64bit/
  xCoreSDK.dll
  xCoreSDK.lib
  xCoreSDK_static.lib
  xCoreSDK_Upgrade.dll
  xCoreSDK_Upgrade.lib
  xCoreSDK_Upgrade_static.lib
  xMateModel.lib              # 64-bit Release only

lib/Windows/Debug/64bit/
  xCoreSDK.dll
  xCoreSDK.lib
  xCoreSDK_static.lib
  xCoreSDK.pdb
  xCoreSDK_Upgrade.dll
  xCoreSDK_Upgrade.lib
  xCoreSDK_Upgrade_static.lib
  xMateModeld.lib             # 64-bit Debug only
```

For 32-bit, replace `64bit` with `32bit` in the path; there is no xMateModel library for 32-bit.

### Linux

```
lib/Linux/x86_64/
  libxCoreSDK.so.{version}
  libxCoreSDK.a
  libxMateModel.a
  libxCoreSDK_Upgrade.so.0.1
  libxCoreSDK_Upgrade.a

lib/Linux/aarch64/
  libxCoreSDK.so.{version}
  libxCoreSDK.a
  libxCoreSDK_Upgrade.so.0.1
  libxCoreSDK_Upgrade.a
```

## Verify

After extraction, run CMake configure at the repository root. CMake will warn if libraries are missing.

## Release Notes (Maintainers)

See [scripts/RELEASE.md](../scripts/RELEASE.md) for packaging and publishing instructions.
