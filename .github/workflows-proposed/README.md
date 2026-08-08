# Move these into `.github/workflows/` to activate

These two workflow files enable fast-forward (`--ff-only`) merges via the
marketplace action [sequoia-pgp/fast-forward](https://github.com/sequoia-pgp/fast-forward).
They live here instead of `.github/workflows/` only because the OAuth token
Claude Code pushes with lacks the `workflow` scope GitHub requires to create
files in that directory — a credential limitation, not a design choice.

To activate (needs a human's own credentials, e.g. locally):

```bash
git mv .github/workflows-proposed/fast-forward.yml \
       .github/workflows-proposed/fast-forward-check.yml .github/workflows/
git rm .github/workflows-proposed/README.md
git commit -m "ci: activate fast-forward workflows"
```

(Or recreate each file at `.github/workflows/<name>` via the GitHub web
editor and delete this directory.)

Once this directory is gone, CLAUDE.md's "Fast-forward merges" section —
added in the same PR and written for the activated state — is accurate.
