# Security and responsible disclosure

**Project:** UR5e AI Motion Planner
**Maintainer:** Xamdamov Abu Bakr

## Reporting a vulnerability

Please avoid publishing exploitable issues or sensitive controller details in public Issues. Use GitHub's **Private vulnerability reporting** for this repository, if enabled. Otherwise, contact the maintainer privately through their GitHub profile to arrange a secure disclosure channel. Do not attach network credentials, live controller addresses, controller logs or personal information to public reports.

## Robotics and safety boundaries

This is experimental research software, **not safety-rated control software**. Workspace checks, approximate arm reach analysis and approval prompts are application-level controls; they do not replace a robot's certified protective functions, risk assessment, emergency stop, safeguarding, or qualified operator supervision.

The software-stop command relies on network connectivity and controller response. Startup and direct `/home` may trigger homing without the AI-task approval dialog after checks pass. Always verify the physical installation before LIVE execution; initial evaluation should be performed in an isolated, supervised environment.

## Confidentiality

The published `ROBOT_IP` is a placeholder and must be configured for local operation. The tracked `limits.py` file contains original setup-specific HOME, workspace and motion parameters, and the technical PDF intentionally preserves those values. Review these details before redistribution. Do not commit edited controller IPs, secrets, site-specific paths, saved execution records or diagnostic reports. The `.gitignore` covers `approval/` and `reports/`, but **does not hide edits to already tracked files** such as `limits.py`.

## Scope of validation

The author reports physical testing of the original setup with the **Ollama `gpt-oss:20b`** model. Alternative model tags and new deployments require separate evaluation. Software-level syntax checks and offline tests do not establish safe robot operation.
