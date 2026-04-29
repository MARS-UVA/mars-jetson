# Webpage Setup Documentation
This site is created using MKDOCS and pushed to Github using the Github Pages action.

## REQUIRED PACKAGES
python3
mkdocs
mkdocs-material
ghp-import
venv

## TESTING
Create a virtual environment with:
```python3 -m venv venv```
```source venv/bin/activate```

Then, run:
```mkdocs serve```

This creates a local webpage for testing view purposes.

## DEPLOYING
Run the following command:
```mkdocs gh-deploy```

This updates the branch **gh-pages** and GitHub Actions will automatically begin to publish the new website.