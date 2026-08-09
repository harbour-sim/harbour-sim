# Move these into `.github/workflows/` to activate

These files enable fast-forward (`--ff-only`) merges via the marketplace
action [sequoia-pgp/fast-forward](https://github.com/sequoia-pgp/fast-forward),
and wire the resulting merges into the deploy pipeline. They live here
instead of `.github/workflows/` only because the OAuth token Claude Code
pushes with lacks the `workflow` scope GitHub requires to create or update
files in that directory — a credential limitation, not a design choice.

- `fast-forward.yml`, `fast-forward-check.yml` — new files.
- `deploy.yml` — **replaces** the existing `.github/workflows/deploy.yml`.
  It adds a `workflow_run` trigger so a fast-forward merge (pushed with the
  default `GITHUB_TOKEN`, which the recursion guard exempts from triggering
  `on: push`) still deploys the site — see the comment in the file, and
  CLAUDE.md's "Fast-forward merges" section, for why no PAT is needed.

To activate (needs a human's own credentials, e.g. locally):

```bash
git mv .github/workflows-proposed/fast-forward.yml \
       .github/workflows-proposed/fast-forward-check.yml .github/workflows/
git mv -f .github/workflows-proposed/deploy.yml .github/workflows/deploy.yml
git rm .github/workflows-proposed/README.md
git commit -m "ci: activate fast-forward workflows"
```

(Or recreate/overwrite each file at `.github/workflows/<name>` via the
GitHub web editor and delete this directory.)

Once this directory is gone, CLAUDE.md's "Fast-forward merges" section —
added in the same PR and written for the activated state — is accurate.
