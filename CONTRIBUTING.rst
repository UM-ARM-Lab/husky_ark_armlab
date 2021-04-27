============
Contributing
============

Issue Reports
=============

If you experience bugs or in general issues with the package, please file an
issue report on our `issue tracker`_.


Code Contributions
==================

Submit an issue
---------------

Before you work on any non-trivial code contribution it's best to first create
an issue report to start a discussion on the subject. This often provides
additional considerations and avoids unnecessary work.

Create an environment
---------------------

Before you start coding we recommend to install Miniconda_ which allows
to setup a dedicated development environment named ``husky-ark`` with::

   conda create -n husky-ark python=3

Note we like doing development in Python3 (and most tools only work in
Python3, but the project must also run in Python2.7 to work on the Husky).

Then activate the environment ``husky-ark`` with::

   conda activate husky-ark
   conda install -c conda-forge pyscaffold tox

Clone the repository
--------------------

#. `Create a Gitub account`_  if you do not already have one.
#. Fork the `project repository`_: click on the *Fork* button near the top of the
   page. This creates a copy of the code under your account on the GitHub server.
#. Clone this copy to your local disk::

    git clone git@github.com:YourLogin/husky_ark_armlab.git husky_ark_armlab
    cd husky_ark_armlab

#. You should run::

    pip install -U pip setuptools
    pip install -e .[dev]

   Note if you are using ``zsh``, you will need to replace ``.[dev]`` with ``.\[dev\]``.

.. TODO: Remove the manual installation/update of pip, setuptools and setuptools_scm
   once pip starts supporting editable installs with pyproject.toml

#. Install ``pre-commit``::

    pre-commit install

   The husky_ark_armlab project comes with a lot of hooks configured to
   automatically help the developer to check the code being written.

#. Create a branch to hold your changes::

    git checkout -b feature/(descriptive-feature-name)-(parent-branch)

   and start making changes. Never work on the master branch and you should almost
   always branch off develop. An example branch name could be ``feature/convert-yolo-to-xyxy-develop``.

   You can also use ``release/`` or ``hotfix/`` as branch prefixes.

#. Please follow the `Conventional Commits`_ commit message guidelines:

   Specifically, you should have both a commit title and body. Additionally, each
   commit title should start with one of the following prefixes:

   - FEAT: (new feature for the user, not a new feature for build script)
   - FIX: (bug fix for the user, not a fix to a build script)
   - DOCS: (changes to the documentation)
   - STYLE: (formatting, missing semi colons, etc; no production code change)
   - REFACTOR: (refactoring production code, eg. renaming a variable)
   - TEST: (adding missing tests, refactoring tests; no production code change)
   - CHORE: (updating grunt tasks etc; no production code change)

   Please see the existing commits for examples of what this should look like.


#. Start your work on this branch. When you’re done editing, do::

    git add modified_files
    git commit

   to record your changes in Git, then push them to GitHub with::

    git push -u origin feature/my-feature-develop

#. Please check that your changes don't break any unit tests with::

    tox

   (after having installed `tox`_ with ``pip install tox`` or ``pipx``).
   Don't forget to also add unit tests in case your contribution
   adds an additional feature and is not just a bugfix.

   To speed up running the tests, you can try to run them in parallel, using
   ``pytest-xdist``. This plugin is already added to the test dependencies, so
   everything you need to do is adding ``-n auto`` or
   ``-n <NUMBER OF PROCESS>`` in the CLI. For example::

    tox -- -n 15

   Please have in mind that the husky_ark_armlab test suite is IO intensive (once
   someone gets around to writing it that is), so using a
   number of processes slightly bigger than the available number of CPUs is a
   good idea. For quicker feedback you can also try::

    tox -e fast

#. Use `flake8`_/`black`_ to check\fix your code style.
#. Add yourself to the list of contributors in ``AUTHORS.rst``.
#. Go to the web page of your fork, and click
   "Create pull request" to send your changes to the maintainers for review.
   Find more detailed information `creating a PR`_. You might also want to open
   the PR as a draft first and mark it as ready for review after the feedbacks
   from the continuous integration (CI) system or any required fixes.


Troubleshooting
===============

    I've got a strange syntax error when running the test suite. It looks
    like the tests are trying to run with Python 2.7 …

Try to create a dedicated venv using Python 3.6+ (or the most recent version
supported by the package) and use a ``tox`` binary freshly installed in this
venv. For example::

    python3 -m venv .venv
    source .venv/bin/activate
    .venv/bin/pip install tox
    .venv/bin/tox -e all


.. _Cirrus-CI: https://cirrus-ci.com/github/UM-ARM-Lab/husky_ark_armlab
.. _PyPI: https://pypi.python.org/
.. _project repository: https://github.com/UM-ARM-Lab/husky_ark_armlab
.. _Git: http://git-scm.com/
.. _Miniconda: https://conda.io/miniconda.html
.. _issue tracker: https://github.com/UM-ARM-Lab/husky_ark_armlab/issues
.. _Create a Gitub account: https://github.com/signup/free
.. _creating a PR: https://help.github.com/articles/creating-a-pull-request/
.. _tox: https://tox.readthedocs.io/
.. _flake8: http://flake8.pycqa.org/
.. _black: https://pypi.org/project/black/
.. _Conventional Commits: https://www.conventionalcommits.org/en/v1.0.0/
.. _A successful git branching model: https://nvie.com/posts/a-successful-git-branching-model/
