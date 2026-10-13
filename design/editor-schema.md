# Editor schema integration (#172)

Status: Implemented and verified locally

# I. Design Contract

## Objective and scope

Make NoDL v2 documents easy to author in VS Code with Red Hat YAML and other YAML Language Server clients.
Issue [#172](https://github.com/ros-tooling/nodl/issues/172) asks for completion, enum suggestions, hover descriptions, inline validation, stable schema URLs, and copyable workspace settings.
This change does not add an editor extension or change runtime validation.

## Required outcomes

1. Published JSON schemas must preserve the canonical YAML schemas' constraints and descriptions, including parameter references.
2. Copyable settings must associate both `*.nodl.yaml` and `*.nodl.yml`, including nested files, with the published node schema.
3. YAML Language Server must offer field and enum completion, show hover descriptions, accept a representative valid document, and report schema violations in checked-in invalid documents.
4. Schema and quickstart documentation must link to the editor setup and explain that semantic checks still require NoDL tooling.

The stable URL will be `https://raw.githubusercontent.com/ros-tooling/nodl/main/nodl_schema/nodl_schema/schemas/nodl.schema.json`.
Assumption: a stable address tracking `main` is sufficient; users can replace `main` with a release tag or commit to pin behavior.
Publication occurs when the files merge to `main`; this task does not publish a branch or change external hosting.
No blocking decisions remain.

# II. Engineering Design

## Proposed code tree

```text
tools/export_editor_schemas.py          Convert explicitly supplied schema files to JSON.
tools/tests/test_editor_schemas.py      Check generated artifacts against canonical sources.
nodl_schema/nodl_schema/schemas/*.json  Published and packaged editor schemas (generated).
examples/editor/settings.json          Copyable YAML Language Server settings.
examples/editor/*.nodl.y*ml             Valid and invalid canonical editor fixtures.
tools/editor/                          Pinned Node dependencies and editor integration tests.
.github/workflows/editor.yml           Run generation checks and editor integration tests.
nodl/doc/editor.md                     Setup, URLs, limitations, and verification commands.
nodl/doc/schema.md                     Link to setup.
README.md                             Link from the existing quickstart.
nodl/doc/index.md                      Register setup in navigation.
nodl_schema/setup.py                   Package generated JSON alongside YAML.
```

## Evidence and architecture

`nodl_schema/nodl_schema/schemas/nodl.schema.yaml` owns the Draft 7 contract and references `parameter.schema.yaml`.
`nodl_schema/schema.py` resolves runtime schemas locally.
`nodl/doc/schema_reference.py` rewrites reference schemas for presentation, so those copies cannot serve editors.
`nodl_schema/setup.py` currently installs YAML schemas.
`README.md` is the project entry point; there is no separate quickstart page.
Add a short Getting started section linking the existing basics tutorial and the editor setup.
YAML Language Server supports `yaml.schemas` associations and schema-driven completion, hover, and validation ([upstream documentation](https://github.com/redhat-developer/yaml-language-server)).

```text
canonical YAML schemas → deterministic exporter → checked-in JSON schemas
                                                       ↓
copyable settings → YAML Language Server → completion / hover / diagnostics
```

The exporter takes input paths and an output directory explicitly.
It changes only supplied schemas' `$id` and `$ref` file names from YAML to JSON, retaining fragments.
It must not duplicate validation policy or read ambient package state.
The JSON pair remains colocated so relative references resolve without a custom provider.
Normal editor operation fetches the public schemas; tests substitute local contents at those same URLs, with unknown requests treated as errors.
This proves reference resolution without requiring the unmerged artifact to exist publicly.

## Alternatives and operational behavior

Direct YAML URLs would avoid generation, but JSON files provide broader schema-client compatibility.
A bundled schema would avoid a second fetch but require reference relocation; two directly converted files preserve the existing ownership boundary.
ReadTheDocs hosting would tie schema availability to the documentation build; raw GitHub hosting requires no new deployment system.
Editors own HTTP caching and failure diagnostics; no new service or retry policy is introduced.
Offline users can download both JSON files together and use a local schema path.
No document migration is required, and removing the settings association rolls back editor integration.
Only public schema metadata is fetched; no node, ROS, or workspace data is sent to NoDL services.

## Test contract and delivery

| Requirement | Evidence |
| --- | --- |
| 1 | Compare each generated JSON object with canonical YAML after only reference/name conversion; validate Draft 7 syntax; check installed package artifacts. |
| 2 | Use the exact checked-in settings with YAML Language Server and test root and nested paths for both suffixes. |
| 3 | Test actual language-service completion for root fields, parameter fields/types and QoS enums, hover for a documented field, no valid-fixture diagnostics, and diagnostics for unknown fields, invalid parameter types, and missing QoS depth. |
| 4 | Build Sphinx with warnings as errors and verify both entry links; include the tested settings with a literalinclude directive. |

Phase 1: implement the explicit exporter, generated files, packaging, and freshness tests.
Gate: artifact tests and package checks pass.
Phase 2 depends on phase 1: add settings, fixtures, pinned editor tests, and CI.
Gate: real YAML Language Server tests pass with no unexpected schema requests.
Phase 3 depends on phase 2: add setup documentation and entry links.
Gate: documentation build and final scoped verification pass.
Each phase can be reverted with its files; no external migration or release automation changes are required.
Work remains serialized because tests and docs share the generated schema URLs and settings.
Stop if editor compatibility requires schema policy changes or duplicated semantic validation.

## Final audit

Requirements 1–4 are implemented in the paths listed above.
Generated JSON copies retain the canonical constraints and documentation, and both files are included in the built wheel's Python package and ament share directory.
The exact workspace settings and fixtures drive 10 passing YAML Language Server checks.
The setup page includes those tested files directly and is linked from the schema reference, navigation, and README Getting started section.

Verification:

- `nodl/doc/.venv/bin/python -m pytest -q tools/tests/test_editor_schemas.py nodl/doc`: 29 passed.
- `PYTHONPATH=$PWD/nodl_schema AMENT_PREFIX_PATH=/tmp/nodl-editor-wheel nodl/doc/.venv/bin/python -m pytest -q nodl_schema/test`: 359 passed, one optional pyright test skipped.
- `npm test --prefix tools/editor`: 10 passed.
- `uv build --wheel nodl_schema --out-dir /tmp/nodl-editor-wheel`: passed; archive contents match the generated schemas in both installation locations.
- `nodl/doc/.venv/bin/sphinx-build -b html -W --keep-going nodl/doc /tmp/nodl-editor-docs`: passed.
- Repository Python, JSON, YAML, and Markdown formatter/checker groups were run on the affected files.

Equivalent implementation choice: README had no quickstart section, so a short Getting started section links the existing basics tutorial and editor setup.
No material contract deviations.
Public URL availability remains a post-merge check; local tests deliberately avoid claiming that the unmerged files are already published.
Existing unrelated workspace changes were preserved.
