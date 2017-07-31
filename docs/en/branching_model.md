# Development Model in Autoware
## Development Workflow
To assure traceablity in our code we follow this development process:
* If you are working on a feature/bug/new-release => create an issue first
* create a branch, work on the feature/bug/new-release, refer in at least one commit to the issue #number
* open pull request for given feature/bug/new-release and fill out pull request template. 

This will assure that we know where the feature originated from and the implementation will be linked to the feature request (or bug report). Otherwise there is absolutely no traceability.
Secondly, with the pull request template it will be easier for the reviewer(s) to understand the changes and we can later on convert "Steps to Reproduce" into integration tests.

## Branching model for Autoware development

In order to keep efficient open-source development of Autoware, we ask all developers to comply with the branching model described below.
On this model, we maily use five branches - master, develop, release, feature, and bugfix.

### Development
In general, developers should work on the "develop" branch.
When you develop new functions, please check out a new branch, "feature/[branch_name]", from the "develop" branch, and modify the code there.
After the modificaiton, please send a pull request to the "develop" branch.

### Release
This situation is true of only the persons in charge of releasing Autoware.
Once we complete some major development, we make a new release.
For the release work, please checkout a new branch, "release/[1.xx.yy], from the "develop" branch, and modify the code there.
After the modification, please send a pull request: "xx" in version of release/[1.xx.yy] is increased when checked out from the "develop" branch, and yy is also increased when bug fixes are done.
Finally, we merge this branch to the master branch, attaching a version tag.

### Bugfix
If we encounter bugs in the "master" branch after the release, we check out the "bugfix" branch from the "master" branch and fix the bugs.
This branch is merged to each corresponding branch - master, release, and develop.
After the merge, the version of the master and the release branches is increased.

Reference for the git-flow model
- http://nvie.com/posts/a-successful-git-branching-model/
