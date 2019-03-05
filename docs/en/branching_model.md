## Development Workflow

To assure traceability in our code we follow this development process:
* If you are working on a feature/modify-node, create an issue first
* Create a branch, work on the feature/modify-node, refer in at least one commit to the issue #number
* Open a pull request for the given feature/modify-node and fill out the pull request template.

This will assure that we know where the feature originated from and the implementation will be linked to the feature request (or bug report). Otherwise there is absolutely no traceability.
Secondly, with the pull request template it will be easier for the reviewer(s) to understand the changes and we can later on convert "Steps to Reproduce" into integration tests.

## Branching Model for Autoware Development

In order to keep efficient open-source development of Autoware, we ask all developers to comply with the branching model described below.
On this model, we mainly use six branches - master, develop, feature, release, experimental, and hotfix.

### master

This is the latest stable version of Autoware.

### develop and feature

In general, developers should NOT work on the "develop" branch.
When you develop new functions, please check out a new branch, "feature/[branch_name]", from the "develop" branch, and modify the code there.
After the modification, please send a pull request to the "develop" branch.

### release

This situation is true of only the persons in charge of releasing Autoware.
Once we complete some major development, we make a new release.
For the release work, please checkout a new branch, "release/[1.xx.yy], from the "develop" branch, and modify the code there.
After the modification, please send a pull request: "xx" in version of release/[1.xx.yy] is increased when checked out from the "develop" branch, and yy is also increased when bug fixes are done.
Finally, we merge this branch to the master branch, attaching a version tag.

### experimental

If your contribution is not a new feature but is a change to one of the existing branches, please checkout a new branch, "experimental/[branch_name]", from the corresponding branch. Please discuss with other contributors if this change can be merged to the original branch.

### hotfix

If we encounter bugs in the "master" branch after the release, we check out the "hotfix" branch from the "master" branch and fix the bugs there.
This branch is merged to each corresponding branch - master, release, and develop.
After the merge, the version of the master and the release branches is increased.

Reference for the git-flow model
- http://nvie.com/posts/a-successful-git-branching-model/
