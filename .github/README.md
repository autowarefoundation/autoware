# Overall CI Infrastructure

## Machine Types

### Standard GitHub-hosted runners

- [Documentation](https://docs.github.com/en/actions/using-github-hosted-runners/using-github-hosted-runners/about-github-hosted-runners#standard-github-hosted-runners-for-public-repositories)

These runners are utilized by the majority of the workflows.
They are free to use for public repositories, with a concurrency limit of 20 jobs per organization.

**Listed specs:**

| vCPU | RAM   | Storage (SSD) |
| ---- | ----- | ------------- |
| 4    | 16 GB | 14 GB         |

**Note:** While the official documentation lists 14 GB of storage, the actual available storage is approximately 73 GB.

### AWS CodeBuild runners

- [AWS CodeBuild Documentation](https://docs.aws.amazon.com/codebuild/latest/userguide/action-runner.html)

These runners are employed for workflows that require more resources and are funded by the Autoware Foundation budget.

**Relevant machine types:**

| Instance Type   | Memory | vCPUs | Price per Minute |
| --------------- | ------ | ----- | ---------------- |
| arm1.large      | 16 GiB | 8     | $0.015           |
| general1.medium | 7 GB   | 4     | $0.01            |
| general1.large  | 15 GB  | 8     | $0.02            |
| general1.xlarge | 72 GiB | 36    | $0.0798          |
| gpu1.small      | 15 GB  | 4     | $0.05            |

**Sources:**

- [Compute images supported with the CodeBuild-hosted GitHub Actions runner](https://docs.aws.amazon.com/codebuild/latest/userguide/sample-github-action-runners-update-yaml.images.html)
- [AWS CodeBuild pricing](https://aws.amazon.com/codebuild/pricing/)

## Key workflows and their runners

| Workflow                           | Trigger               | Runner         |
| ---------------------------------- | --------------------- | -------------- |
| build-and-test (cuda)              | merge to main         | general1.large |
| build-and-test-daily               | daily on main         | github-std     |
| build-and-test-daily-arm64         | daily on main         | arm1.large     |
| build-and-test-differential        | PR update             | github-std     |
| build-and-test-differential (cuda) | PR update             | general1.large |
| build-and-test-differential-arm64  | PR update (arm label) | arm1.large     |

## Additional notes

- We use [`taskset`](https://manpages.ubuntu.com/manpages/jammy/man1/taskset.1.html) from GNU Coreutils to limit the number of cores utilized by build processes. This is done to prevent overloading the self-hosted runners.
  - The number of cores is limited to `vCPU count - 1`.
