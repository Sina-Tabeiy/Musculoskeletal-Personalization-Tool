# E4 — Subject-Robust Threshold Calibration: Pre-Registration

**Document ID:** E4-PREREG-005
**Protocol:** `strict_loso_v1` (protocol hash `d39d831aec8cb1fe34968b7400d1ae2297b2ef4b1f63fa822d731100ff3b5ec8`)
**Baseline:** E0, frozen strict LOSO (23 folds)
**Status:** PROPOSED — NOT APPROVED — NOT IMPLEMENTED — NOT EXECUTED
**Revision:** 5 (incorporates the decisions of 2026-11-01 and 2026-11-02, including the approved promotion criteria U1–U3)
**Revision date:** 2026-11-02
**Supersedes:** E4-PREREG-004 (on-disk revision 4); E4-PREREG-003 (on-disk revision 3); E4-PREREG-002 (on-disk revision 2); E4-PREREG-001 (response-only draft; never written to disk)

---

## 1. Document control

### 1.1 What this document is

A pre-registration for E4, a **saved-score threshold-aggregation experiment**. E4 does not
retrain models, does not change features, labels, preprocessing, alarm semantics, or the
outer-fold partition, and does not re-run E0. It reuses the frozen E0 OOF scores (to build
per-subject thresholds) and the frozen E0 outer-test scores (to evaluate them).

### 1.2 What this document is not

- It is **not** an approval to execute E4. The promotion criteria are now approved, but
  execution authorization (U6) is not.
- It is **not** a claim that any E4 result exists.
- It **does** record the two numerical promotion requirements, which the user approved on
  2026-11-02: the absolute compliance floor `C_A1 >= 15/23` (U1) and the relative improvement
  requirement `DeltaC >= 2/23` (U2). Approval of the promotion rule does not authorize
  execution and does not declare any promotion.
- It does **not** authorize reading outer-test prediction scores for any purpose beyond the
  evaluation that Phase B specifies.

### 1.3 File provenance

Revision 2 was the first on-disk version; the earlier draft was delivered in conversation
only. Revision 3 applied the decisions of 2026-11-01 and the G0 execution-order alignment.
Revision 4 applied the decisions of 2026-11-02 and reconciled the remaining research
documents with the fixed execution order and the approved arm roster. Revision 5 records the
approved promotion criteria U1–U3, reconciles the E0 completion status across the project
documentation, and fixes the normative experiment output roots. The files modified in producing
revision 5 are this document, `FINAL_RESEARCH_ROADMAP.md`, `PROPOSED_EXPERIMENTS.md`,
`EXPERIMENT_PRIORITY_MATRIX.csv`, `RESEARCH_IDEAS.md`, `UNRESOLVED_ISSUES.md`, and
`PROJECT_STATE_AUDIT.md`. No experiment code, E0 artifact, or configuration file was created or
modified.

### 1.4 Decisions incorporated in revisions 3 and 4

**Revision 3 (2026-11-01):**

| # | Decision | Where |
|---|---|---|
| D1 | `DeltaC` is retained as a primary operational comparison; the fixed 0.20 relative cutoff is withdrawn and not replaced by another arbitrary number; the exact absolute and relative promotion requirements remain undecided | Sections 13.2–13.4; Appendix B U1–U2 |
| D2 | The **paired outer-fold bootstrap (descriptive)** is retained as descriptive uncertainty only, never a promotion gate | Section 13.6 |
| D3 | The sensitivity allowance is narrowed from 5 to 2 percentage points against E0, giving a minimum of `8/141 = 5.674%` | Section 13.5 |
| D4 | More than 50% suppressed folds is a **Mandatory Review Flag**, not a promotion gate | Section 13.7 |
| D5 | The high-level execution order is `E0 -> E1 -> E2 -> E4`; E4 remains an independent frozen-score experiment | Section 5.2; G0 amendment in `FINAL_RESEARCH_ROADMAP.md` |
| D6 | The canonical EDF identity is `(subject_id, edf_filename)`; `(fold_id, edf_code)` is artifact-scoped only | Section 4.7 |
| D7 | Phase A reads only an explicit field-level allowlist; outer-test-derived fields and prediction contents are unreadable | Section 10.1 |
| D8 | The weighted-quantile endpoint clamp is part of the E4 specification, with pseudocode and a numerical example | Section 8.1 |

**Revision 4 (2026-11-02):**

| # | Decision | Where |
|---|---|---|
| D9 | **Arm roster is exactly A1–A4** (A1 equal-subject median, A2 equal-subject P75, A3 hours-weighted median, A4 hours-weighted P75). The conformal and maximum-threshold alternatives in older documents are superseded and inactive | Section 6; `PROPOSED_EXPERIMENTS.md` E4 |
| D10 | **U5 resolved:** the occupancy moving-block bootstrap is **NOT PART OF E4**; no moving-block or temporal-block interval is computed or reported | Section 12; Appendix B U5 |
| D11 | **Sensitivity guardrail approved:** the maximum absolute decrease is 2 percentage points versus E0, i.e. at least `8/141`; it is a non-inferiority guardrail, not a claim of clinical adequacy | Section 13.5; Appendix B U3 |
| D12 | **Bootstrap terminology fixed:** the procedure is called only the **paired outer-fold bootstrap (descriptive)**; no synonym is used | Section 13.6 |
| D13 | **Phase A allowlist precision:** `n_ok`, `n_failed`, `n_skipped` are permitted `_status.json` integrity fields; `manifest.json` and `fold_results[*]` permissions are enumerated; runtime and warning fields remain forbidden | Section 10.1 |
| D14 | **Execution isolation:** every experiment uses a dedicated output root; E1/E2 must never write to the E0 root; E4 uses a separate output root and reads E0 without modifying it | Sections 1.5, 5.1, 14, 15.1; `PROPOSED_EXPERIMENTS.md` §0 |
| D15 | **E0 manifest integrity:** all 46 fold-level artifacts are verified against the E0 manifest; the manifest stores no OOF hashes, so E4 records OOF input hashes itself | Sections 5.1, 10.2, 11.1 |
| D16 | **Roadmap reconciliation:** the roadmap, catalogue, matrix, ideas, and unresolved-issues documents are reconciled to the fixed order and approved criteria; E5 follows E4; no withdrawn numeric criterion remains active | `FINAL_RESEARCH_ROADMAP.md`; `PROPOSED_EXPERIMENTS.md`; companion files |
| D17 | **Promotion criteria unchanged at revision 4:** U1 (absolute compliance) and U2 (relative improvement) remained **PENDING USER APPROVAL**; no numeric replacement was invented | Sections 13.3, 13.4, 13.8; Appendix B U1–U2 |

*D1 and D17 record the pre-revision-5 state only. U1 and U2 were approved in revision 5
(D18–D19), which supersedes their "undecided"/"pending" status while preserving this history.*

**Revision 5 (2026-11-02) — promotion criteria approved and E0 status reconciled:**

| # | Decision | Where |
|---|---|---|
| D18 | **U1 approved:** A1 must reach at least **15 compliant outer folds of 23**, i.e. `C_A1 >= 15/23 = 65.22%`. A fold is compliant only if `FAR <= 0.25/h` **and** `occupancy <= 0.05`. U1 is an independent absolute floor | Section 13.3; Appendix B U1 |
| D19 | **U2 approved:** A1 must improve fold compliance by at least **two folds** over E0, `DeltaC = C_A1 - C_E0 >= 2/23 = 0.0869565` (8.70 percentage points). With `C_E0 = 17/23`, passing U2 requires `C_A1 >= 19/23 = 82.61%`. U1 remains an independent floor; passing both therefore effectively requires at least 19/23 under the current baseline. This is a pre-specified operational criterion, not a statistical-significance or generalization claim | Section 13.4; Appendix B U2 |
| D20 | **U3 confirmed, U4 and U5 preserved:** the 2-percentage-point sensitivity non-inferiority guardrail (at least `8/141`), the fixed `E0 -> E1 -> E2 -> E4 -> E5` order, and the exclusion of the occupancy moving-block bootstrap are unchanged | Sections 5.2–5.4, 13.5, 12; Appendix B U3–U5 |
| D21 | **U6 unchanged:** approval of U1–U5 does **not** authorize E4. Status remains PROPOSED / NOT APPROVED / NOT IMPLEMENTED / NOT EXECUTED | Section 5.3; Appendix B U6 |
| D22 | **E0 status reconciled:** E0 is complete — 23/23 valid folds, 22 newly computed and one valid reused fold (`chb01`), zero failed folds. Historical 1/23 statements are retained only as dated history | `PROPOSED_EXPERIMENTS.md`; `EXPERIMENT_PRIORITY_MATRIX.csv`; `PROJECT_STATE_AUDIT.md`; `FINAL_RESEARCH_ROADMAP.md`; `UNRESOLVED_ISSUES.md` |
| D23 | **Normative output roots:** E0 `strict_loso_v1/`; E1 `strict_loso_v1/experiments/E1/`; E2 `strict_loso_v1/experiments/E2/`; E4 `strict_loso_v1/experiments/E4/`; E5 `strict_loso_v1/experiments/E5/`. No root may equal, contain, or nest inside another; E4 reads E0 without modifying it | Sections 1.5, 5.1, 14, 15.1; `PROPOSED_EXPERIMENTS.md` §0.8; `FINAL_RESEARCH_ROADMAP.md` §8.4 |

### 1.5 Hard boundaries carried into execution (if ever approved)

1. E0 artifacts are **read-only**. No E0 file, hash, or threshold may be rewritten.
2. Phase A computes and seals thresholds from **OOF data only**; Phase B verifies the seal
   and only then loads outer-test predictions.
3. No threshold, aggregation rule, or metric definition may be changed after any outer-test
   score is inspected.
4. No fold may be silently dropped. Every fold is reported, including failures.
5. Only `far_025` decides fold-level compliance. All nine operating points are reported.
6. Only A1 is potentially promotable. A2, A3, A4 are descriptive.
7. Phase A reads only the fields enumerated in Section 10.1. Outer-test-derived fold fields
   (`test`, `warnings`), every prediction table, and every aggregate performance table are
   unreadable in Phase A; prediction `.npz` files are hashed but never opened.
8. **Execution isolation.** E4 writes only to its own dedicated output root. The E0 root is
   read-only: no E4 output path may equal, contain, or sit inside the E0 root, and no E0 file
   may be rewritten. A shared or nested output root is a stop condition (Section 17), not a
   warning.
9. **E0 input integrity.** Before Phase A computes any threshold, the 46 fold-level E0
   artifacts (23 fold JSONs and 23 prediction `.npz` files) are verified against the E0
   manifest. Because the manifest stores no OOF hashes, E4 records SHA-256 for every
   `fold_*_oof.npz` and for `training_meta.json` itself, and re-verifies those hashes before
   applying any sealed threshold (Section 11.1).

---

## 2. Background and frozen E0 reference

### 2.1 Verified E0 reference values

These are **historical reference results produced by the completed E0 run**, verified from the
frozen aggregate tables during this revision. They are not E4 findings (see Section 3.3).

**Run state.** 23/23 fold artifacts are present, valid, and protocol-hash compatible; zero
failed folds. `folds/_status.json` records `n_ok = 22`, `n_failed = 0`, `n_skipped = 1`.
The skipped fold is `chb01`, which was reused from the 2026-10-29 run
(`fold_chb01.json`, `created_utc = 2026-10-29T11:31:03Z`, `runtime_sec = 1335.391`).
Reuse provenance is documented in Section 5.1. `n_skipped = 0` is **not** required.

**Primary operating point (`far_025`), pooled over 23 held-out subjects:**

| Quantity | Value | Source |
|---|---|---|
| Eligible seizures | 141 | `results/operating_points.csv`, row `far_025` |
| Detected seizures | 10 / 141 = **7.09%** (Wilson 3.90–12.56%) | same |
| Event-level FAR | **0.242/h** (Poisson 0.211–0.277) | same |
| Occupancy | **2.02%** | same |
| Alert events (true) | 229 (12) | same |
| Zero-detection subjects | 17 / 23 | same |
| Median lead time | 1002.5 s | same |

**Fold-level compliance at `far_025`** — compliant iff `FAR <= 0.25/h` **and**
`occupancy <= 0.05`:

| | Count | Fraction |
|---|---|---|
| E0 compliant folds (`C_E0`) | **17 / 23** | 0.7391 |
| E0 non-compliant folds | 6 / 23 | 0.2609 |

Non-compliant folds, with verified values: `chb01` (FAR 1.308/h, occ 0.1499), `chb05`
(0.272/h, 0.0082), `chb06` (0.748/h, 0.0426), `chb10` (0.406/h, 0.0258), `chb13` (0.789/h,
0.0492), `chb24` (0.498/h, 0.0602). Source: `results/per_fold_operating_points.csv`.

**`chb01`, the subject most often cited as the transfer failure (corrected values):**

| Quantity | `far_025` | `occ_05` | Source |
|---|---|---|---|
| AUC, raw probabilities | 0.4254 | — | `results/per_subject.csv` |
| AUC, causal smoothed | 0.4196 | — | same |
| Seizures detected | 1 / 6 | 1 / 6 | `results/per_fold_operating_points.csv` |
| Occupancy | **14.99%** | **30.83%** | same |
| FAR | **1.308/h** (0.968–1.729) | 1.362/h | same |
| Earliest lead | 1195 s | 1195 s | same |

Correction notice: earlier text quoted `chb01` as "occupancy 30.8%, FAR 1.31–1.82/h". The
30.8% figure belongs to `occ_05`, not `far_025`; and 1.82/h is `chb01`'s value at the higher
points `occ_15` / `far_100`, not at `far_025`. The values above are the verified ones and
supersede the earlier wording.

**Discrimination.** Pooled AUC 0.5742 (raw) and 0.5938 (smoothed); macro AUC 0.5623 and
0.5857. Source: `results/summary.md`. Pooled AUC concatenates scores from 23 different fold
models and is descriptive only.

### 2.2 What the reference does and does not establish

Establishes:

- Calibration is a further constraint beyond ranking for a subset of folds: 17/23 subjects
  have AUC above 0.5, yet only 6 subjects detect any seizure at `far_025`, and six subjects
  with smoothed AUC at or above 0.65 detect none (`results/RESULTS_SUMMARY.md:172-188`).
- Pooled threshold transfer is systematically poor for at least `chb01`, which is the
  motivating case for a subject-robust aggregation rule.

Does not establish:

- That a per-subject aggregation rule will improve compliance. That is E4's open hypothesis.
- That any arm will preserve sensitivity.
- That 80% sensitivity is reachable. At `far_025`, E0 detects 10/141; the union of all nine
  pre-specified points is 60/141. Calibration cannot close that gap.

---

## 3. Scope

### 3.1 In scope

- Build per-training-subject thresholds from frozen OOF scores at all nine operating points.
- Aggregate them into four pre-registered arms (A1–A4).
- Seal the aggregated thresholds before any outer-test score is loaded.
- Apply the sealed thresholds to the frozen outer-test smoothed scores.
- Report compliance, sensitivity, FAR, occupancy, lead time, and uncertainty for E0-ref and
  A1–A4 at all nine operating points.

### 3.2 Out of scope

- Any feature, label, preprocessing, or model change.
- Any retraining, re-fitting, recalibration, or early stopping.
- Any correction of cross-subject score-scale shift (see Section 9).
- Any per-subject adaptation on the held-out patient, any post-hoc threshold rescue, or any
  re-selection of the aggregation rule after seeing outer results.
- Any external cohort, any new dataset, any claim of 80% sensitivity.
- Any implementation of E4 itself. This document is a plan only.

### 3.3 Relationship to E0 results

- **E4 generates no new E0 results.** The E0 run is complete and frozen; E4 never re-runs it,
  never re-fits a model, and never recomputes an E0 threshold from scratch.
- **E4 uses the frozen E0 results as its reference baseline.** `E0-ref` in every E4 table is a
  re-read of the frozen E0 per-fold and pooled numbers, not a new computation.
- **Every E0 value quoted in this document is a historical reference value**, verified from
  `strict_loso_v1/results/` and `strict_loso_v1/folds/`. None is an E4 finding.
- Where E4 changes a number, it is because E4 applied a different sealed threshold; the
  comparison is always against the frozen E0 value, never against a re-derived one.
- The one E0 recomputation E4 performs is the Phase B reproduction check (Section 11.3),
  whose only purpose is to prove the frozen artifacts are internally consistent before E4
  thresholds are applied. If that check fails, E4 halts; it cannot revise E0 or itself.

---

## 4. Frozen definitions (imported, not redefined)

E4 imports the following from the frozen implementation. No definition below is new.

### 4.1 Primary operating point

`far_025`: target FAR 0.25 false-alarm events per interictal hour, with an auxiliary
occupancy ceiling of 0.15 used only while constructing the FAR threshold
(`strict_loso_v1/config.json:134-136`). All nine points (`occ_01`, `occ_02`, `occ_05`,
`occ_10`, `occ_15`, `far_010`, `far_025`, `far_050`, `far_100`) are reported; only `far_025`
decides compliance.

### 4.2 Occupancy

`occupancy = FP / n_interictal`, where `FP` counts interictal sequences whose alarm predicate
is true, and the denominator is every interictal (`y == 0`) sequence of the held-out subject.
The alarm predicate is `smoothed >= threshold` (`strict_loso_v1/strict_loso.py:486`;
`strict_loso_v1/strict_common.py:922`).

### 4.3 FAR and alarm events

Alarm events are formed within one EDF only: consecutive alerting sequences belong to the
same event when the next alert starts no more than 60 s after the previous alert ends;
refractory period is 0 s; events never merge across EDF boundaries
(`strict_common.py:477-505`; `config.json:112-118`). An event is a **true alarm** iff its span
contains at least one preictal (`y == 1`) sequence. `FAR = event_fp / interictal_hours`, with
the denominator equal to the total duration of interictal sequences in hours
(`strict_common.py:924-925`, `471-475`).

### 4.4 Smoothing

Causal trailing mean over `k = 12` sequences: **360 s of included sequence duration**
(12 × 30 s) and a **330 s start-to-start lookback** (`start_j >= start_i - 330`), reset at EDF
boundaries (`config.json:102-111`; `strict_common.py:407-443`). The 360 s figure is the total
duration of the averaged sequence set; the 330 s figure is the span between the earliest and
current sequence start. These are not interchangeable, and E4 never describes the lookback as
360 s.

E4 does **not** re-smooth. It uses `smoothed` as stored in the frozen artifacts, and verifies
that value against a recomputation from `prob` as an integrity check only (Section 11.2).

### 4.5 Lead time and the 1,228 s envelope

Lead time per seizure key is `onset - end` of a fired positive sequence:
earliest `onset - min(end[fired])`, latest `onset - max(end[fired])`
(`strict_common.py:948-950`).

The positive label interval is `[onset - 1200, onset - 300)` (`config.json:34`), but sequences
are grid-aligned and are labelled positive if they *overlap* that interval. Verified from
`training_meta.json`: of 3,616 seizure-sequence links, 88 start 2–28 s **before**
`onset - 1200`, and no linked positive sequence ends after `onset - 300`. The maximum start
offset is therefore **1,228 s**, i.e. positive-sequence start times lie in
`[onset - 1228, onset - 300)`.

This is a **start-offset envelope, not lead-time support**. Because lead is measured from
sequence end, and no positive sequence ends after `onset - 300`, the maximum attainable lead
is under 1,200 s (≈1,198 s for a sequence starting at `onset - 1228`). Consequently E0's
diagnostic `lead > 1228` (`config.json:156`) is unreachable for eligible seizures and cannot
be used as evidence for a 1,228 s lead claim.

**Disposition:** the 1,228 s figure is recorded as a verified metadata property of positive
sequence starts. It is **excluded from the E4 specification**: E4 uses it for no threshold,
no eligibility rule, no lead metric, and no decision.

### 4.6 Seizure eligibility and multi-key rows

The frozen catalog is built by `strict_common.py:179-205` from every metadata row that carries
seizure identifiers: key `(edf_filename, seizure_id, onset_sec)`, with later rows appended to
that key's `rows` list. Eligibility is exactly the frozen rule: every distinct annotated key
with at least one preictal sequence in the held-out patient's data; **no duration filter and no
minimum-lead filter** (`config.json:166`). E4 imports this catalog without adding filters,
re-keying, merging, or renumbering, and verifies that its per-fold eligible counts match E0
(e.g. `chb01` = 6).

**Multi-key rows are real, not hypothetical.** Verified from `training_meta.json`: 38 of the
3,570 positive rows carry more than one identifier — 30 carry two and 8 carry three
(`seizure_ids` / `seizure_onsets` are parallel lists produced by `Auditing_V4.py:892-907`,
which links every seizure whose preictal span overlaps the sequence). The catalog therefore
places such a row in the row list of **each** linked key.

Evaluator behaviour is per key: a key is detected iff at least one row in *its* row list fires
(`strict_common.py:940-950`). Hence one alerting row that links two keys counts towards both
keys' detections. This is inherited E0 semantics, is identical for E0-ref and A1–A4, and is
**not modified in E4**. Two related facts are recorded to prevent misreading:

- The 141 eligible keys are distinct `(edf, seizure_id, onset)` triples; the 3,570 positive
  rows are fewer than the 3,616 links because of this duplication.
- Overlapping preictal intervals are merged **only** to decide sequence labels
  (`Auditing_V4.py:501-517`); merging never removes or merges keys. Distinct keys remain
  distinct for seizure-level detection, even when their intervals overlap.

No reconciliation problem was found between metadata and evaluator semantics; this item is
resolved and does not block E4.

### 4.7 EDF identity and `edf_code` reconstruction

`edf_code` is **not** a globally stable EDF identifier. In the frozen loader it is a run-local
inverse index produced by

```python
edf_unique, edf_code = np.unique(edf_names, return_inverse=True)   # strict_common.py:176-177
```

computed over the full metadata array (111,130 rows, 662 unique filenames). Codes are assigned
in sorted-filename order, so the mapping is independent of row order once the metadata file is
fixed. Consequences that E4 must respect:

1. The only canonical EDF identity is `(subject_id, edf_filename)`. `fold_subject_id` is
   **not** a valid OOF row identity: it identifies the held-out outer subject, whereas an OOF
   artifact contains rows only from the 22 outer-training subjects. Verified for
   `fold_chb01_oof.npz`: 106,462 OOF rows spanning 22 distinct metadata `subject` values, with
   `chb01` absent. Where an artifact-scoped local identifier is needed, use `(fold_id,
   edf_code)` or `(artifact_id, edf_code)`. Such an identifier is valid **only within that
   artifact** and must never be joined across folds or runs.
2. E4 must **not** regenerate EDF codes per subject, per fold, or per artifact. It must
   reconstruct the mapping once, with the exact frozen call above, from the full
   `training_meta.json` array.
3. E4 must **not** join records across folds or runs using a bare `edf_code`.
4. Before any EDF-boundary-dependent operation, E4 verifies that the artifact's stored
   `edf_code` equals `reconstructed_inverse[row_index]` elementwise. On mismatch, the affected
   operation is **blocked**; E4 does not guess or repair.

**Verification performed during this revision:** the reconstruction above was applied to
`training_meta.json` and compared with the stored `edf_code` of every `fold_*_oof.npz`
(23 files, 2,444,860 rows). All rows matched exactly, no file mismatched. The procedure is
therefore deterministic for the frozen artifacts.

### 4.8 Threshold functions, flags, and suppression

Occupancy thresholds use the exact order statistic with `nextafter(..., +inf)`, so ties cannot
push OOF occupancy above target; flags `below_resolution`, `ties_undershoot`,
`target_covers_all_rows` (`strict_common.py:659-705`). FAR thresholds use an exact union-find
sweep over descending OOF scores because event merging makes FAR non-monotone in the
threshold; the auxiliary occupancy ceiling is 0.15; if the target is unattainable without
suppressing interictal alarms, the threshold is `nextafter(max_interictal, +inf)` and the flag
`far_target_unattainable_without_suppressing_interictal_alerts` is set
(`strict_common.py:708-886`; `config.json:135`). All returned flags are preserved verbatim in
E4 outputs.

**Suppression is strict.** Because the alarm predicate is `smoothed >= threshold`, a threshold
equal to the maximum interictal OOF score still alerts on the row holding that maximum.
Complete interictal suppression therefore requires

```
threshold > max(OOF interictal smoothed scores)
```

(equivalently `threshold >= nextafter(max, +inf)`). Equality is **not** suppression; it is
recorded as `boundary_contact`. E4 never describes a `>=`-equal threshold as suppressing all
interictal alarms, and never treats `nextafter`-style near-equality as suppression unless the
strict inequality holds.

### 4.9 Undefined metrics

E4 preserves E0's `NaN` semantics and never substitutes 0 or 1:

| Metric | Undefined when | Source |
|---|---|---|
| Sequence precision | `tp + fp == 0` | `strict_common.py:915` |
| Sequence F1 | `2tp + fp + fn == 0` | `strict_common.py:917` |
| Event precision | `event_alerts == 0` | `strict_common.py:991-994` |
| Wilson interval | denominator 0 | `strict_common.py:547-555` |
| AUC / AUPRC | fewer than two classes | `strict_common.py:586-590` |
| FAR | interictal exposure 0 h | `strict_common.py:924-925` |
| Occupancy | no interictal rows | `strict_common.py:922` |

Undefined values are reported as `null` in JSON and "undefined (reason)" in tables. An
undefined **secondary** metric never invalidates a fold and never converts a valid fold into
an inconclusive one. Only the metrics that define fold compliance (FAR and occupancy) and the
pooled primary quantities (seizure counts, sensitivity) are required for a fold to be
conclusive.

---

## 5. Preconditions

### 5.1 E0 artifact validity

E4 may begin only if all of the following hold:

1. 23 fold JSONs exist with `status == "ok"` and `protocol_hash` equal to
   `d39d831aec8cb1fe34968b7400d1ae2297b2ef4b1f63fa822d731100ff3b5ec8`.
2. Each fold has both `fold_<subject>_oof.npz` and `fold_<subject>_predictions.npz`.
3. Zero failed folds. `n_skipped` may be 1; `n_skipped = 0` is **not** required.
4. The reused fold is documented: `chb01` was carried over from the 2026-10-29 run
   (`created_utc = 2026-10-29T11:31:03Z`), has `status == "ok"`, matches the current protocol
   hash, and its artifacts are present. Its reuse is recorded in this document and must be
   carried into the E4 output metadata. Reuse is accepted because the fold is hash-compatible
   with the current protocol; it is still reported as a provenance limitation.
5. The frozen aggregate report records complete, duplicate-free pooling: `pooled_rows =
   111130`, `duplicate_pooled_rows = 0`, `pooled_rows_equal_dataset_rows = True`
   (`results/summary.md`). EDF-code agreement is verified in Phase A for OOF artifacts and in
   Phase B for prediction artifacts (Section 4.7); Phase A does not parse prediction files.
6. No non-finite values in stored OOF scores. Prediction-score finiteness is checked in
   Phase B (Section 11.2), because Phase A does not parse prediction files.
7. **Output-root separation.** The E4 output root is dedicated to E4 and is neither equal to
   nor nested inside the E0 root (`strict_loso_v1`). The E0 root remains read-only. This is a
   prospective execution precondition; it cannot be satisfied by a path that shares
   `folds/`, `results/`, or `logs/` with E0.
8. **E0 manifest integrity.** Every `fold_results[*]` entry in `manifest.json` supplies
   `json.sha256` and `predictions_npz.sha256` for that fold. Recomputing SHA-256 over the 46
   fold-level files must match all 46 recorded values, and `n_folds_ok = 23`,
   `n_folds_failed = 0`, `failures = []`. Only the manifest fields enumerated in Section
   10.1 are read.
9. **OOF input hash recording.** The E0 manifest does **not** contain OOF hashes. E4 therefore
   records SHA-256 for each of the 23 `fold_*_oof.npz` files and for `training_meta.json` at
   the start of Phase A, stores them in the seal record, and re-verifies them before applying
   any threshold (Section 11.1). This is a recording requirement, not a claim that the manifest
   already stores those hashes.

Conditions 1–6 and 8 were checked during revisions 3 and 4 and hold for the frozen E0 inputs.
Condition 7 is a prospective execution requirement, and condition 9 is an E4-side recording
requirement; neither can be satisfied by editing an E0 file.

### 5.2 G0 status and execution order — AGREED 2026-11-01; extended 2026-11-02

The user decision of 2026-11-01 fixes the high-level execution order as:

```
E0 -> E1 -> E2 -> E4 -> E5
```

- **E0 is complete and frozen.** It is not re-executed by E4.
- **E1 and E2 are completed before E4; E5 follows E4.** The rationale is to perform the planned diagnostic
  investigations (E1: scale-invariant spectral representation; E2: multi-scale causal temporal
  context) before evaluating the threshold-aggregation experiment. Their purposes and
  completion criteria are taken from `FINAL_RESEARCH_ROADMAP.md` §6.2–6.3 and
  `PROPOSED_EXPERIMENTS.md` E1–E2 sections, not inferred from their names. E5's baseline is
  fixed by the E4 promotion outcome (Section 5.4).
- **E4 remains independent.** It uses the original E0 OOF and outer-test scores and their
  existing provenance. E1/E2 results must not modify E0 scores, E4 thresholds, aggregation
  weights, arm definitions, or selection rules. E4 is not tuned after inspecting E1/E2
  findings. If E1/E2 reveal a problem that requires changing E4 inputs or methodology, E4 is
  not silently modified: the issue is documented and a separately approved protocol revision
  or experiment is requested.
- The G0 diagnostic branching (strong / mixed / weak baseline) is preserved in
  `FINAL_RESEARCH_ROADMAP.md` §8.2 and remains a description of E0. The branch-conditional
  **reordering** is superseded for scheduling by this fixed sequence.

**Reconciled in revision 4 (2026-11-02).** Revision 3 recorded four roadmap clauses as
conflicts and retained them as historical text. Revision 4 reconciles those clauses in
`FINAL_RESEARCH_ROADMAP.md` by amendment notes; the historical wording is preserved, but it
is explicitly superseded for scheduling and is not active:

1. §6.4's clause moving E4 ahead of E1/E2 is annotated as superseded for scheduling.
2. §8.1's conditional reordering to `E0 -> E4 -> E5 -> E1 -> E2 -> ...` is annotated as
   superseded; the table row for E4 now depends on completed E1/E2.
3. §8.3's permission to run E4 while E2 is fitting is removed from the active schedule; E5
   is also scheduled after E4, not concurrently with E2.
4. G1's "if not already run, E4" and the G0 mixed-baseline advice to take the calibration
   branch before E1/E2 are annotated as superseded; E1 and E2 are run before E4 regardless
   of G0 branch.

U4 is therefore **closed** by revision 4: the sequence is `E0 -> E1 -> E2 -> E4 -> E5`, with
E5's baseline selected only after the E4 promotion outcome is known (Section 5.4). E4 is
placed after E1 and E2 in the high-level sequence, but that placement implies no execution
date and no authorization: E4 remains NOT APPROVED and NOT EXECUTED.

### 5.3 Execution authorization

E4 may not be implemented or run without explicit overall authorization to execute it
(Appendix B U6). The promotion rule is fully specified — absolute floor U1 `C_A1 >= 15/23`,
relative improvement U2 `DeltaC >= 2/23` (effective `C_A1 >= 19/23`), and sensitivity guardrail
U3 `>= 8/141` — but approving a promotion rule is not the same as authorizing execution. The
descriptive role of the paired outer-fold bootstrap, the arm roster A1–A4, the exclusion of U5,
the fixed execution order, and the output-root isolation are also decided; no E4 authorization
follows from any of them.

### 5.4 E5 placement

E5 (alarm post-processing) follows E4. Its baseline is fixed per the pre-registered E4
promotion outcome: E0 scores if no E4 arm is promoted, or the promoted E4 threshold bundle if
one is. E5 is not run concurrently with E2 or before the E4 outcome is recorded. This
subsection records placement only; it does not authorize E5.

---

## 6. Arms

| Arm | Weighting | Quantile `p` | Promotable | Description |
|---|---|---|---|---|
| `E0-ref` | pooled OOF threshold from `fold_*.json` | — | No (baseline) | Frozen E0 operating point; read, never recomputed |
| **A1** | equal subject weight (1 each) | 0.50 | **Yes** | Equal-subject midpoint median — primary arm |
| A2 | equal subject weight (1 each) | 0.75 | No | Equal-subject midpoint P75 (more conservative) |
| A3 | interictal-hours weight | 0.50 | No | Hours-weighted midpoint median |
| A4 | interictal-hours weight | 0.75 | No | Hours-weighted P75 |

Only A1 may be promoted. A2–A4 exist to show the sensitivity/conservativeness trade-off and
to test whether the result depends on the equal-subject or equal-hour convention.

**Arm roster confirmed in revision 4.** The roster is exactly `E0-ref` plus **A1, A2, A3,
A4**; no fifth arm exists. The 90%-coverage conformal arm and the maximum-threshold arm that
appear in earlier drafts and in older paragraphs of `PROPOSED_EXPERIMENTS.md`,
`EXPERIMENT_PRIORITY_MATRIX.csv`, `RESEARCH_IDEAS.md` (C2), and `UNRESOLVED_ISSUES.md` are
**superseded and inactive**. They were never approved, are not part of this design, are not
reported, and are not eligible for promotion.

Each arm produces one threshold per operating point per fold, from the 22 training subjects of
that fold. Nothing is shared across folds: fold `s`'s thresholds use only fold `s`'s OOF rows,
which come from the 22 subjects other than `s`.

---

## 7. Per-subject threshold construction

For fold `s`, training subject `v` (22 of them), and operating point `op`:

1. Select the OOF rows belonging to `v` (`row_index` from `fold_s_oof.npz`, filtered by
   metadata `subject`). These rows are all rows of `v`; the OOF file for fold `s` contains
   every row of every training subject exactly once.
2. Build `AlarmEvaluator(y_v, edf_code_v, start_v, end_v, max_gap_sec=60)` on those rows.
   This is required because FAR is event-level, and events are EDF-bounded.
3. Compute the threshold with the **frozen** function:

```
occupancy point: t_v(op) = occupancy_threshold(smoothed_v[y_v == 0], target)
far point:       t_v(op) = far_threshold(smoothed_v, y_v, evaluator_v, target, 0.15)
```

   No modification, no smoothing, no re-scaling, no rounding. The returned flags are recorded.
4. Record, descriptively: `t_v`, achieved OOF occupancy of `v` at `t_v`, achieved OOF FAR of
   `v` at `t_v` (FAR points only), `v`'s interictal hours, `v`'s eligible/detected seizure
   counts, and whether `t_v > max(smoothed_v[y_v == 0])` (subject-level suppression, strict
   inequality per Section 4.8).
5. Weights: `w_v = 1` for A1/A2; `w_v = interictal_hours(v)` for A3/A4, computed inside Phase A
   from the same OOF rows (`sum(end - start for y == 0) / 3600`).

Constraints:

- Every training subject has positive interictal exposure (verified range 15.59–153.99 h,
  total 896.33 h across all 23 subjects), so no zero or non-finite weight can arise.
- Only OOF rows of training subjects are used. The held-out subject contributes **zero** rows.
- OOF labels used here are training-patient labels. They are the same labels E0 used to
  calibrate its pooled thresholds; no held-out label is read in Phase A.

---

## 8. Aggregation

### 8.1 Midpoint weighted quantile (normative definition for all arms)

No E4 aggregation code exists as of this revision. This subsection is therefore the normative
definition that any future implementation must reproduce exactly. The endpoint clamp described
below is **part of the algorithm**, not an optional or cosmetic adjustment.

Given values sorted as `t_(1) <= ... <= t_(n)` with weights `w_(i) > 0` and `n >= 2` (every E4
fold has `n = 22` training subjects, so the degenerate `n = 1` case cannot arise):

```text
W    = sum_i w_(i)
C_i  = sum_{j <= i} w_(j)
m_i  = (C_i - 0.5 * w_(i)) / W        # midpoint plotting position, i = 1..n
m_1  = 0                              # endpoint clamp (normative)
m_n  = 1                              # endpoint clamp (normative)

t(p):
  if p <= m_1: return t_(1)
  if p >= m_n: return t_(n)
  find the smallest i with m_i >= p
  if m_i == p: return t_(i)
  return t_(i-1) + (p - m_(i-1)) / (m_i - m_(i-1)) * (t_(i) - t_(i-1))
```

Because `m_1 = 0` and `m_n = 1`, no extrapolation is ever performed. Weights are normalised by
`W`, so multiplying every weight by a common constant does not change `m_i`. The algorithm acts
on order statistics; there is no random tie-breaking.

**Numerical example (n = 3, `w = (1, 2, 1)`, `t = (1, 2, 4)`).**
`W = 4`, `C = (1, 3, 4)`; the clamped positions are `m = (0, 0.5, 1)`.

- `p = 0.50` equals `m_2` exactly and returns `t_(2) = 2`.
- `p = 0.75` lies between `m_2 = 0.5` and `m_3 = 1`, so it returns
  `2 + (0.75 - 0.5)/(1 - 0.5) * (4 - 2) = 3`.
- `p = 0.05 <= m_1` returns `t_(1) = 1`; `p = 0.95 >= m_3` returns `t_(3) = 4`.
- The clamp is material: with the unclamped midpoint position `m_3 = 0.875`, `p = 0.75`
  would return `3.333`. The normative definition returns `3`.

**Effect at the arm quantiles.** For the 22-subject folds, the unclamped positions are
`m_1 = 0.5/22 = 0.0227` and `m_22 = 21.5/22 = 0.9773`; after clamping they are 0 and 1. Both
`p = 0.50` and `p = 0.75` lie strictly inside `[m_2, m_21]` (`0.0682 <= p <= 0.9318`), so the
clamp does not change A1 or A2 at their arm quantiles. The equal-weight identities in
Section 8.2 therefore hold under the clamped definition exactly as stated.

### 8.2 Equal-weight sanity checks (n = 22)

With `w_i = 1`, the unclamped midpoint positions are `(i - 0.5) / 22`; the clamp sets `m_1 = 0`
and `m_22 = 1`. The arm quantiles sample only interior positions, where the unclamped formula
applies:

- `p = 0.50`: `m_11 = 10.5/22 = 0.4773`, `m_12 = 11.5/22 = 0.5227`; interpolation at 0.5 is
  exactly halfway, so A1 returns the **mean of the 11th and 12th order statistics**, i.e. the
  conventional median.
- `p = 0.75`: `m_17 = 16.5/22 = 0.75` exactly, so A2 returns the **17th order statistic**.

These identities are asserted as unit checks before any threshold is sealed. If either fails,
Phase A halts.

### 8.3 Weighted behaviour (A3/A4)

Weights are strictly positive and non-uniform. A subject's influence is proportional to its
OOF interictal hours. A3/A4 are descriptive only; a large difference between A1 and A3
indicates that a few long recordings dominate the hours-weighted aggregate, and must be
reported as such rather than interpreted as robustness.

### 8.4 Canonical form and seal

Thresholds are serialized as canonical JSON: UTF-8, keys sorted, floats emitted with
shortest round-trip `repr`, newline `\n`, no NaN/Infinity (Phase A requires finite
thresholds; a non-finite threshold is a stop condition, not something to encode). The SHA-256
of those exact bytes is the seal. The seal record also stores the SHA-256 of every input
artifact and of `training_meta.json`, plus the protocol hash and config hash.

---

## 9. Score-scale limitation

Per-subject thresholds are computed on **inner-model OOF scores** and applied to
**final-model scores** on a subject no inner model ever saw. The final model is fitted on all
22 training subjects; each inner model was fitted on 21. Their probability outputs are not
guaranteed to share a scale.

Implications, all of which E4 reports rather than repairs:

- A threshold that achieved the target FAR on a subject's OOF scores may overshoot or
  undershoot on the final-model test scores. E0 already exhibits this; E4 cannot remove it.
- The aggregation is an empirical robustness heuristic, not a calibrated guarantee. No
  coverage statement (for example "90% of subjects") is claimed.
- A negative E4 result is ambiguous between "subject thresholds do not transfer" and "the
  final-model score scale differs from every inner model's scale". The ambiguity is reported,
  not resolved post hoc.
- No post-hoc normalization, rank transform, quantile mapping, or per-fold rescaling of test
  scores is permitted. Any such step would be an unregistered methodological change.

---

## 10. Phase A — compute and seal thresholds (OOF only)

### 10.1 Allowed and forbidden inputs (field-level)

Phase A may read only the fields listed in the "Phase A allowed" column. Everything else is
unreadable, including every outer-test-derived metric, count, threshold, warning, or
performance summary. Opening a forbidden file, or reading a forbidden field, is a stop
condition (Section 17), not a warning.

| Input | Phase A allowed | Phase A forbidden | Phase B |
|---|---|---|---|
| `folds/fold_*.json` | `fold`, `status`, `protocol_version`, `protocol_hash`, `config_sha256`, `feature_meta_sha256`, `seed`, `created_utc`, `threshold_freeze_utc`, `runtime_sec`, `strictness`, `artifacts` (existence check only) | `counts`, `inner`, `normalization`, `model`, `thresholds`, `test`, `warnings` | all fields |
| `folds/_status.json` | `protocol_version`, `protocol_hash`, `protocol_fingerprint`, `started_utc`, `n_ok`, `n_failed`, `n_skipped`, per-fold `status`, per-fold `runtime_sec` (the last only to document the reused `chb01` fold, Section 5.1.4) | per-fold `warnings` | all fields |
| `manifest.json` | top level: `protocol_version`, `protocol_hash`, `expected_protocol_hash`, `generated_utc`, `n_folds_ok`, `n_folds_failed`, `failures`, `fold_results` (subfields only, below); per fold: `json.path`, `json.sha256`, `predictions_npz.path`, `predictions_npz.sha256` | `command`, `environment`, `seed`, `config`, `inputs`, `outputs`; per-fold `runtime_sec`; per-fold `warnings`; any performance field | all fields |
| `folds/fold_*_oof.npz` | `row_index`, `prob`, `smoothed`, `y`, `edf_code`, `start`, `end` | any other array | same fields |
| `training_meta.json` | `subject`, `edf`, `seq_id`, `start`, `end`, `label`, `seizure_ids`, `seizure_onsets`; the full array is read **for `edf_code` reconstruction only** | any held-out label, feature, or score entering a threshold or weight | same fields |
| `training_y.npy` | labels at the OOF `row_index` only | any held-out row | same fields |
| `results/summary.md` | only the pooling-integrity fields `pooled_rows`, `duplicate_pooled_rows`, `pooled_rows_equal_dataset_rows`, plus the strictness-assertion table | AUC, sensitivity, FAR, occupancy, detection counts, leads, zero-detection lists, and every per-subject/per-fold performance table | reproduction check |
| `results/operating_points.csv`, `per_fold_operating_points.csv`, `per_subject.csv`, `seizure_level.csv`, `strict_results.json`, `RESULTS_SUMMARY.md` | nothing for any decision | all contents | reproduction check only |
| `folds/fold_*_predictions.npz` | SHA-256 over the file's raw bytes; the archive is **not opened** | arrays, shape, statistics, or any parsed content | after seal verification |
| `folds/fold_*_predictions.csv`, `folds/fold_*_alarms.csv` | nothing | all contents | not required |

Required clarifications:

1. **No outer-test-derived information may influence any Phase A decision.** Phase A must not
   read outer-test metrics, outer-test counts, or performance summaries for threshold
   selection, aggregation, arm selection, weighting, or any other decision. The `test` and
   `warnings` fields of `fold_*.json` and the per-fold `warnings` of `_status.json` exist and
   contain outer-test performance information; they are explicitly excluded.
2. **Prediction files are hash-only in Phase A.** SHA-256 is computed over the raw file bytes
   by a byte-stream hash. The `.npz` archive is not opened, no array is loaded, and no shape,
   statistic, or header is inspected. The hash is used only to prove the file did not change.
3. **Phase B access follows the seal.** Phase B may open outer-test prediction files only after
   the threshold bundle has been hashed, sealed, and its seal verified (Sections 11.1–11.3), and
   only for the evaluation defined in Section 11.4.
4. **The frozen aggregate tables are not a Phase A input.** The threshold bundle is reproducible
   from the OOF artifacts and metadata alone. `results/summary.md` is consulted only to
   cross-check the three named pooling-integrity values against a recomputation from OOF rows;
   no performance value is read.
5. **Held-out filenames are used only for identity reconstruction.** Reconstructing `edf_code`
   requires the full metadata array, including held-out subjects' filenames. That use is
   filename-only and is required to reproduce the frozen loader's code assignment exactly
   (Section 4.7); no held-out label, feature, or score enters any threshold or weight.
6. **Every precondition is verifiable from this allowlist.** `_status.json` `n_ok`,
   `n_failed`, and `n_skipped` cover the fold-status precondition (Section 5.1.3); the
   manifest's `n_folds_ok`, `n_folds_failed`, `failures`, and the 46 per-fold
   `json.sha256` / `predictions_npz.sha256` values cover the artifact-integrity precondition
   (Sections 5.1.1–5.1.4 and 5.1.8). Per-fold `runtime_sec` in the manifest is **forbidden**,
   and per-fold `warnings` is forbidden in both the manifest and `_status.json`; no
   precondition requires either. If a future precondition would need a currently forbidden
   field, the precondition must be revised explicitly rather than the allowlist broadened
   silently.

### 10.2 Steps

```
A0  Verify preconditions (Section 5.1) using only the Phase A allowlisted fields
    (Section 10.1). Confirm that the E4 output root is not equal to, does not
    contain, and is not nested inside the E0 root. Abort on any failure.
A0b Verify E0 manifest integrity: recompute SHA-256 over the 23 fold JSONs and the
    23 prediction .npz files and compare with fold_results[*].json.sha256 and
    fold_results[*].predictions_npz.sha256 (46/46). Do not open the prediction
    archives; hash their raw bytes only.
A0c Record SHA-256 for every fold_*_oof.npz and for training_meta.json. The E0
    manifest has no OOF hashes; these recorded values are E4 inputs, not manifest
    fields, and are carried into the seal record.
A1  Reconstruct edf_code: np.unique(edf_names, return_inverse=True) over the full
    training_meta.json array, exactly as the frozen loader does.
A2  For each fold s:
      load OOF only; assert row_index unique and equal to the union of the 22 training
      subjects' rows; assert y == training_y[row_index]; assert times == metadata;
      assert edf_code == reconstructed_inverse[row_index]; assert all scores finite.
      Do not open fold_*_predictions.npz or read the test/warnings fields.
A3  For each fold s, each training subject v, each operating point op:
      compute t_v(op) with the frozen functions (Section 7); record flags and diagnostics.
A4  Aggregate each arm (A1-A4) at each op using Section 8.1.
A5  Assert Section 8.2 equal-weight identities; record threshold distributions
      (min, P25, median, P75, max, IQR) and A1-vs-A3 difference.
A6  Canonicalize and hash (Section 8.4), including the A0b/A0c input hashes.
A7  Write the threshold bundle and the seal record. No metric on outer-test scores is computed.
A8  Print a manifest of files read, files hashed-only, and files written.
```

### 10.3 Outputs (proposed; not created)

- `threshold_bundle.json` — per fold, per op, per arm: aggregated threshold, the 22 `t_v`,
  weights, per-subject diagnostics, flags.
- `threshold_seal.json` — bundle hash, input hashes, protocol/config hashes, timestamps.
- `phase_a_log.txt`.

### 10.4 Failure handling

- Any precondition failure → abort before computing anything.
- Any non-finite threshold, any zero/negative weight, or any failed identity assertion → abort.
- A subject-level `far_target_unattainable_without_suppressing_interictal_alerts` flag is not a
  failure; it is recorded, and the subject's threshold is the frozen suppression threshold.
- Phase A never inspects outer-test values, so no failure in Phase A can be "fixed" by
  consulting them.

---

## 11. Phase B — verify, reproduce, evaluate

### 11.1 Seal verification

1. Recompute the canonical bytes and SHA-256 of the bundle; must equal `threshold_seal.json`.
2. Recompute SHA-256 of every input artifact and compare with the seal: each
   `fold_*_oof.npz`, `training_meta.json`, and the 46 manifest-listed fold JSON and prediction
   `.npz` files. The E0 manifest stores no OOF hashes, so the seal's OOF values are the
   recorded A0c hashes. Prediction archives remain unopened at this step.
3. Abort on any mismatch. No threshold is applied after a failed seal check.

### 11.2 Outer-test artifact integrity

For each fold:

- `row_index` is exactly the held-out subject's rows, once each.
- `y`, `start`, `end` equal the metadata values for those rows.
- `edf_code` equals the reconstruction from Section 4.7.
- `prob` and `smoothed` are finite.
- Optional consistency check: recompute the causal trailing mean from `prob` with the frozen
  function and compare with stored `smoothed` (tolerance 1e-12). This is an integrity check,
  not a re-evaluation; it cannot change any threshold.

Any failure → halt; the fold is reported as failed, not dropped.

### 11.3 E0 reproduction (required before applying E4 thresholds)

Using the frozen per-fold thresholds from `fold_*.json` (not recomputed), recompute for each
fold the `far_025` and `occ_05` metrics from the stored predictions with the frozen evaluator
and compare against `results/per_fold_operating_points.csv`:

| Field | Tolerance |
|---|---|
| `tp`, `fp`, `tn`, `fn`, `event_tp`, `event_fp`, `event_alerts`, `seizures_detected`, `seizures_eligible` | exact integer equality |
| `occupancy`, `far_per_hour`, `seizure_sensitivity` | absolute 1e-6 |

Also compare pooled `far_025` (`10/141`, FAR 0.242097, occupancy 0.020184) against
`results/operating_points.csv`.

Rules:

- Any mismatch → **halt E4**. The thresholds are not revised, and no E4 metric is reported as
  valid.
- The 1,228 s diagnostic is excluded from this check (Section 4.5). Reproduction compares only
  the fields above.
- Reproduction failures are never "resolved" by changing the protocol or re-running E0.

### 11.4 Apply sealed thresholds

For each fold and arm: `pred = smoothed_test >= T_arm(op)`; evaluate with the frozen
`evaluate_prediction` and the frozen seizure catalog for the held-out subject. Evaluate all
nine operating points for all five rows (E0-ref, A1–A4). Only `far_025` feeds Section 13.

### 11.5 Outputs (proposed; not created)

- `e4_operating_points.csv` — per arm × op × fold, plus pooled rows.
- `e4_per_subject.csv` — per subject × arm × op.
- `e4_seizure_level.csv` — per seizure key × arm at `far_025`, with lead times.
- `e4_thresholds_applied.json` — the sealed thresholds actually applied.
- `e4_summary.md` — narrative with all compliance, guardrail, and uncertainty numbers.
- `e4_manifest.json` — inputs, hashes, environment, timestamps.

---

## 12. Metrics and uncertainty reporting

Per arm and operating point, pooled and per subject:

- Seizure-level: eligible, detected, missed, sensitivity with Wilson 95%; mean/median/earliest
  lead; zero-detection subjects.
- Event-level: alerts, true alerts, event precision with Wilson 95%, FAR/h with exact Poisson
  95%.
- Sequence-level: sensitivity, specificity, precision, F1 (undefined per Section 4.9
  reported as undefined).
- Occupancy (exact definition, Section 4.2).
- Discrimination: pooled and macro AUC / AUPRC for raw and smoothed scores; pooled values are
  descriptive across fold models.

Uncertainty rules:

- Wilson for proportions, exact Poisson for FAR, row-bootstrap for pooled AUC, and the
  per-subject bootstrap for macro AUC, seed 42, exactly as E0. E0's confidence intervals are
  not widened, narrowed, or replaced. The per-subject bootstrap for macro AUC is E0's existing
  uncertainty method and is **not** the paired outer-fold bootstrap of Section 13.6.
- Compliance is a count of folds, reported as an exact integer `k/23` for E0-ref and for every
  E4 arm, together with `C_X = k/23` and `DeltaC = C_A1 - C_E0` (absolute value reported). No
  independent-binomial interval is attached, because folds are not independent.
- Fold-level transitions between E0 and A1 are reported separately as **improved**
  (E0 non-compliant -> A1 compliant), **regressed** (E0 compliant -> A1 non-compliant), and
  **unchanged** (same compliance status). The counts sum to 23.
- Seizure-level agreement between E0 and A1 at `far_025` is reported where the frozen artifacts
  support it: keys detected by E0 but missed by A1, and keys detected by A1 but missed by E0.
- **Occupancy moving-block bootstrap: NOT PART OF E4** (decided 2026-11-02, Appendix B U5).
  No moving-block, temporal-block, or sequence-block interval on occupancy is computed,
  reported, or referenced as an E4 result. Occupancy is reported as a point estimate with its
  fold-level values, exactly as in E0.
- All numbers are reported together. No sensitivity figure appears without its FAR, occupancy,
  lead time, and uncertainty.

---

## 13. Promotion criteria

**The numerical promotion criteria are approved (2026-11-02).** The absolute compliance floor
(U1), the relative improvement requirement (U2), and the sensitivity non-inferiority guardrail
(U3) are normative. The descriptive uncertainty analysis and the mandatory review flag are not
promotion gates. Approval of this promotion rule is **not** an approval to execute E4: execution
authorization remains **U6 — NOT APPROVED**. Section 13.8 summarizes.

### 13.1 Fold compliance `J_f`

At `far_025`, for arm `X` and fold `f`:

```
J_f(X) = 1  iff  FAR_f(X) <= 0.25 /h  AND  occupancy_f(X) <= 0.05
```

Both conditions are required, with no tolerance. Neither a 10% nor a 20% tolerance is applied.
A fold with an undefined FAR or occupancy cannot be compliant; this cannot occur in the frozen
data because every subject has interictal exposure (minimum 15.59 h).

### 13.2 Compliance counts and `DeltaC`

```
C_X    = (1/23) * sum_f J_f(X)     # fraction of compliant folds
DeltaC = C_A1 - C_E0
```

Verified reference: `C_E0 = 17/23 = 0.7391`.

`DeltaC` is retained as a **primary operational comparison metric**. The following are
mandatory reporting requirements, independent of which numerical promotion thresholds are
eventually approved:

1. The exact compliant-fold count `k` and `C_X = k/23` for `E0-ref` and for each of A1–A4.
2. `DeltaC = C_A1 - C_E0`, reported with its sign and its absolute value.
3. Fold-level transitions between E0 and A1, reported separately:
   - **Improved:** E0 non-compliant -> A1 compliant.
   - **Regressed:** E0 compliant -> A1 non-compliant.
   - **Unchanged:** both have the same compliance status.
   The three counts sum to 23, and the fold identities are listed.
4. The full count table below, so any observed count can be located without recomputation.

| Compliant folds `k` | `C_X = k/23` | `DeltaC` vs `C_E0 = 17/23` | Promotion status under U1/U2 |
|---|---|---|---|
| 15/23 | 0.6522 | −0.0870 | Meets U1 floor only; fails U2 |
| 16/23 | 0.6957 | −0.0435 | Meets U1 floor only; fails U2 |
| 17/23 | 0.7391 | 0.0000 | Meets U1 floor only; fails U2 |
| 18/23 | 0.7826 | 0.0435 | Meets U1 floor only; fails U2 |
| 19/23 | 0.8261 | 0.0870 | Meets U1 and U2 |
| 20/23 | 0.8696 | 0.1304 | Meets U1 and U2 |
| 21/23 | 0.9130 | 0.1739 | Meets U1 and U2 |
| 22/23 | 0.9565 | 0.2174 | Meets U1 and U2 |
| 23/23 | 1.0000 | 0.2609 | Meets U1 and U2 |

The table starts at 15/23 because that is the approved absolute floor (U1, Section 13.3); any
count below 15 is also reported and fails U1. The approved relative requirement (U2, Section
13.4) requires at least `2/23` improvement, so under `C_E0 = 17/23` it requires at least 19/23;
U1 is not removed even though U2 is currently stricter. Two bounds follow from `C_E0 = 17/23`:
`DeltaC` cannot exceed `6/23 = 0.2609` (only six folds are non-compliant in E0) and cannot fall
below `-17/23 = -0.7391`.

### 13.3 Absolute compliance requirement (APPROVED 2026-11-02: `C_A1 >= 15/23`)

**Why an absolute requirement is needed.** A relative rule alone can promote an arm that still
leaves individual subjects outside the targets. For example, a move from 17/23 to 21/23 compliant
folds is a positive `DeltaC = +4/23 = 0.1739` while two subjects remain non-compliant. An absolute
requirement states the minimum fraction of folds that must satisfy `J_f = 1` under A1,
independent of E0, and therefore encodes the deployment-relevant claim directly.

**Approved requirement (U1).** A1 must achieve at least **15 compliant outer folds out of 23**:

```
C_A1 = k_A1 / 23 >= 15/23 = 0.6522 = 65.22%
```

A fold is compliant only if **both** conditions hold:

```
FAR_f(A1) <= 0.25 alarms/hour   AND   occupancy_f(A1) <= 0.05
```

This is an absolute minimum compliance floor. It is independent of U2. It is an operational
promotion criterion, not a statistical-significance or generalization claim.

**Relationship to E0 and U2.** `C_E0 = 17/23 = 73.91%`, so the 15/23 floor is already met by E0.
U1 therefore functions as a non-inferiority floor and cannot by itself demonstrate improvement;
U2 (Section 13.4) supplies the improvement requirement. Because U2 requires at least 19/23 under
the current baseline, passing both requirements effectively requires at least 19/23 compliant
folds. U1 is not removed merely because U2 is currently stricter.

**Status of `14/23` and the 60% example.** The earlier draft's `C_A1 >= 14/23` is **withdrawn**
and is not active. It had no provenance beyond the roadmap's illustrative "for example at least
60%" wording (60% of 23 is 13.8, rounded to 14), and no pre-existing project file recorded the
literal `14/23`. The approved U1 value is 15/23; no other absolute value is active.

**Reporting.** Regardless of the verdict, the exact compliant-fold counts for E0-ref and every
arm and the full count table in Section 13.2 are always reported.

**Status.** **APPROVED 2026-11-02** (Appendix B, U1). Approval does not authorize E4 execution.

### 13.4 Relative improvement requirement (APPROVED 2026-11-02: `DeltaC >= 2/23`)

**Why a relative requirement is needed.** The absolute requirement can be satisfied by E0 before
E4 runs. A promotion rule must also establish that the aggregation actually improves on the
frozen baseline; otherwise A1 would be promoted for matching, not for fixing, the transfer
failure. The relative requirement is therefore part of the promotion rule and is stated in terms
of `DeltaC = C_A1 - C_E0`.

**Approved requirement (U2).** A1 must improve fold compliance by at least **two outer folds**
relative to E0:

```
DeltaC = C_A1 - C_E0 >= 2/23 = 0.0869565 = 8.70 percentage points
```

With the frozen baseline `C_E0 = 17/23 = 73.91%`, U2 requires:

```
C_A1 >= (17 + 2)/23 = 19/23 = 0.8261 = 82.61%
```

`DeltaC` is a difference in fold-compliance proportions, not a percentage change relative to the
E0 value. The requirement is a **pre-specified operational promotion criterion**. It is not
described as statistically significant and is not proof of generalization.

**Relationship to U1.** U1 remains an independent absolute floor of 15/23 (Section 13.3). U2 is
a separate relative-improvement requirement. Because U2 is stricter under the current E0
baseline, passing both effectively requires at least 19/23 compliant folds. U1 is retained even
though U2 is currently stricter.

**Withdrawn, not replaced by the old value.** The earlier draft used a fixed `0.20` relative
cutoff on `DeltaC`. That value is **withdrawn** and is not active. Its provenance was
`PROPOSED_EXPERIMENTS.md` E4 (historical 2026-10-30 text), which mixed a per-fold +20% tolerance
with a 20-percentage-point compliance improvement; the +20% tolerance had already been removed,
and the 20-point cutoff was never ratified. The approved value is `2/23 = 0.0870`; it is a
different, explicitly approved criterion, not a reinstatement of the 0.20 proposal.

**Arithmetic context (not a second criterion).** `DeltaC` is bounded by `[-17/23, +6/23]`. Any
positive value requires at least one E0 non-compliant fold to become compliant with no
regression; a value above `5/23 = 0.2174` would require at least 22/23. These bounds describe
what the approved threshold can and cannot mean.

**Reporting.** `DeltaC` is reported with its sign and absolute value, together with the exact
compliant-fold counts for every arm and the improved/regressed/unchanged transition counts
(Section 13.2).

**Status.** **APPROVED 2026-11-02** (Appendix B, U2). Approval does not authorize E4 execution.

### 13.5 Sensitivity non-inferiority guardrail (2 percentage points — APPROVED 2026-11-02)

**Definition.** Pooled seizure-level sensitivity at `far_025` must satisfy

```
Sensitivity_A1 >= Sensitivity_E0 - 0.02
```

with the same frozen denominator, the same frozen eligibility definition, and the same
seizure-level detection rule as E0. Nothing about the 141-key denominator or per-key detection
is redefined by E4.

**Arithmetic, using the frozen E0 baseline:**

| Quantity | Value |
|---|---|
| E0 detections | 10 / 141 |
| E0 sensitivity | 7.092% |
| Permitted absolute loss | 2.00 pp of 141 = 2.82 seizures |
| A1 floor | `10 - 2.82 = 7.18` detections, i.e. at least **8** |
| A1 floor as a fraction | 8 / 141 = **5.674%** |
| Reference: 7/141 | 4.965% — below the floor, so 8 is the minimum integer |

**Statement.** A1 must detect **at least 8 of 141 seizures (5.674%)** to satisfy the guardrail.
Seven detections (4.965%) fail it.

**Required reporting.** The actual sensitivity and the actual detected-seizure count are always
reported, not only a Pass/Fail verdict. In addition, where the frozen artifacts support it,
report the keys detected by E0 but missed by A1 and the keys detected by A1 but missed by E0
(Section 12).

**Interpretation limits.**

- This is a **sensitivity non-inferiority guardrail relative to E0**. It is **not** evidence of
  clinically adequate sensitivity, and passing it does not mean sensitivity was preserved.
- The 2-percentage-point margin is a **pre-specified operational design choice**. It is not a
  clinically validated threshold and is not statistically derived from E0. E0's 10/141 carries
  a Wilson interval of 3.90–12.56%, and 8/141 lies inside that interval; the guardrail is
  therefore a floor against collapse, not an equivalence test.
- The guardrail applies to A1 only, at `far_025` only. A2–A4 are reported without a guardrail
  verdict.
- The 2 pp value replaces the earlier 5 pp allowance (and the larger loss it implied). The
  margin was **approved on 2026-11-02** as a sensitivity non-inferiority guardrail (Appendix
  B U3, resolved). Approval does not convert it into a clinical-adequacy claim or an
  equivalence test, and it does not authorize E4 execution (U6).

### 13.6 Paired outer-fold bootstrap (descriptive; not a promotion gate)

**Definition.** Resample the 23 fold indices with replacement (`B = 2000`, seed 42), applying
the same resampled indices to both arms, recompute `DeltaC* = C_A1* - C_E0*` on each resample,
and report the percentile 95% interval together with the point estimate `DeltaC`. Because the
same fold indices are used for both arms, the comparison is paired. This procedure has exactly
one name in every E4 document: **paired outer-fold bootstrap (descriptive)**.

**Role (decided 2026-11-01).** The interval is a **supplementary descriptive uncertainty
analysis only**. It is **not** a mandatory Promotion Gate, no promotion verdict uses it, and a
result is not labelled inconclusive solely because the interval overlaps zero. It is reported
alongside `DeltaC` and the improved/regressed/unchanged counts of Section 13.2.

**Terminology.** The procedure is never called an "independent-subject bootstrap", a "subject
bootstrap", a "paired fold-index bootstrap", or any other synonym. It resamples outer folds,
not subjects, and it does not estimate between-patient variability.

**Dependence limitation (must be reported wherever the interval appears).** Outer LOSO folds
are **not** independent observations. Each fold trains on 22 of the 23 subjects, and any two
folds' training sets overlap in 21 of 22 subjects, so every fold shares almost all of its
training data with every other fold. A bootstrap over 23 fold labels therefore resamples
heavily overlapping estimators, not independent draws from a population of patients or
training runs. Consequences:

- The interval reflects fold-to-fold heterogeneity in the frozen cohort. It does **not**
  estimate uncertainty over new patients, new recordings, or new training runs.
- A lower bound above 0 does **not** prove independent-subject generalization, and a lower
  bound overlapping 0 does **not** prove the absence of an effect.
- The interval must never be described as an external-validity or generalization claim.

**Status.** **DECIDED: descriptive only.** The earlier roadmap language tying an overlapping
interval to an "inconclusive" outcome (`PROPOSED_EXPERIMENTS.md` E4, historical 2026-10-30 text)
is not a promotion rule
and does not govern E4. The analysis itself is retained.

### 13.7 Suppression-dominated results — Mandatory Review Flag

**Definition (unchanged).** For fold `s`, the A1 threshold at `far_025` is *suppressing* iff

```
T_A1(s) > max(pooled OOF interictal smoothed scores of fold s)
```

strictly, per Section 4.8. Define `suppression_fraction` as the number of suppressing folds
divided by 23, and `suppression_dominated` as `suppression_fraction > 0.50`.

**Role (decided 2026-11-01).** A `suppression_dominated` result is a **Mandatory Review Flag**,
not a Promotion Gate. If the flag is triggered:

- promotion review is mandatory before any promotion is claimed;
- sensitivity and alarm behaviour in the suppressed folds are reviewed;
- the number and percentage of suppressed folds are reported;
- where the frozen artifacts support it, each suppressed fold is reported with whether it
  produced any outer-test alerts and whether it detected any seizures.

A1 is **not** automatically rejected solely because the flag is triggered. The flag changes the
review procedure; it does not by itself decide the promotion outcome.

**OOF suppression is not outer-test false-alarm behaviour.** Suppression is defined on pooled
OOF interictal scores. It does **not** mean that the corresponding outer-test fold had zero
false alarms. Outer-test alerts, FAR, and occupancy are evaluated and reported independently
for every fold, including suppressed ones.

**Boundary case.** If `T_A1(s)` equals the maximum interictal OOF score, the fold is **not**
suppressing: under `smoothed >= threshold` the maximum-scoring interictal row still alerts.
Such folds are counted as `boundary_contact` and reported separately. They are never folded
into the suppression count.

**Change control.** The suppression definition and its calculation may not be altered without a
documented reason and explicit approval.

Also reported: the number of individual training subjects whose `t_v` suppressed their own
interictal OOF alarms, and the resulting per-subject threshold distribution.

### 13.8 Status summary

| Element | Definition | Status |
|---|---|---|
| Absolute compliance requirement | `C_A1 >= 15/23 = 65.22%`; fold compliant iff `FAR <= 0.25/h` and `occupancy <= 0.05` | **APPROVED 2026-11-02** (U1); independent floor; 14/23 withdrawn |
| Relative improvement requirement | `DeltaC = C_A1 - C_E0 >= 2/23 = 0.0869565` (8.70 pp); effective `C_A1 >= 19/23 = 82.61%` | **APPROVED 2026-11-02** (U2); prespecified operational criterion; 0.20 cutoff withdrawn |
| Sensitivity non-inferiority guardrail | `Sensitivity_A1 >= Sensitivity_E0 - 0.02`, i.e. `>= 8/141` at `far_025` | **APPROVED 2026-11-02** (U3 resolved); non-inferiority only, not clinical adequacy |
| Paired outer-fold bootstrap (descriptive) | 95% interval of `DeltaC` | **DECIDED: descriptive only; not a gate**; single name; dependence limitation mandatory |
| Suppression > 50% of folds | `suppression_fraction > 0.50` | **DECIDED: Mandatory Review Flag; not a Promotion Gate** |
| Occupancy moving-block bootstrap | temporal-block interval on occupancy | **NOT PART OF E4** (U5 resolved 2026-11-02); not computed or reported |
| E4 arm roster | `E0-ref` + A1–A4 | **DECIDED: A1–A4 only**; conformal and maximum-threshold arms superseded and inactive |
| E4 execution authorization | overall approval | **NOT APPROVED** (U6); approval of U1–U5 does not authorize execution |

**Behaviour now that the promotion rule is approved.** The absolute floor (U1), relative
improvement (U2), and sensitivity guardrail (U3) are fully specified and can be evaluated once
E4 is authorized and executed. Until U6 authorization is given and E4 produces results, no
promotion is declared and no arm is described as "passing". If E4 is authorized and run, all
results are reported in full and the verdicts use U1, U2, and U3 exactly as defined above. The
descriptive bootstrap interval and the suppression review flag are reported as descriptive and
review-flag information; they are never turned into pass/fail gates. U1 and U2 are both required
for promotion, and the Mandatory Review Flag must be addressed before a promotion is claimed.

---

## 14. Leakage controls

| Control | Phase A | Phase B |
|---|---|---|
| Held-out patient rows used in any threshold, weight, or decision | Never | Evaluation only |
| Held-out patient labels used in any threshold, weight, or decision | Never | Evaluation only |
| Outer-test scores parsed | Never | After seal verification only |
| Outer-test-derived fold fields (`test`, `warnings`) read | Never | Reproduction check only |
| Prediction `.npz` opened or parsed | Never (raw-byte SHA-256 only) | After seal verification only |
| Aggregate performance tables read | Never for any decision | Reproduction check only |
| E0 thresholds recomputed | Never | Never (read from `fold_*.json`) |
| Feature statistics or model fitted | Never | Never |
| Threshold changed after test inspection | Impossible (sealed) | Forbidden; would require a new pre-registration |
| Aggregate E0 tables written | Never | Never |
| E0 root written or modified | Never | Never |
| E4 output written outside its dedicated root | Never | Never |

Additional controls:

- **Output-root isolation.** E4 writes only to its dedicated output root. That root is not the
  E0 root, is not nested inside it, and does not share `folds/`, `results/`, or `logs/` with
  it. Before Phase A, the resolved paths are compared and an overlap is a stop condition
  (Section 17). The same rule applies to E1, E2, and E5: each uses its own root, and none may
  write into the E0 root.
- **E0 integrity is verified, not assumed.** Phase A checks the 46 manifest-listed fold
  artifacts and records OOF and metadata hashes, because the manifest contains no OOF hashes
  (Sections 5.1.8–5.1.9, 10.2 A0b–A0c). A mismatch stops the run; E0 is never repaired.
- Phase A's field-level allowlist is Section 10.1. Every field not listed there is forbidden,
  including `counts`, `thresholds`, `test`, `warnings`, and all prediction contents.
- The full metadata array is read only to reconstruct `edf_code` from filenames, exactly as the
  frozen loader does. That filename-only use does not place any held-out label, feature, score,
  or statistic into a threshold or weight.
- Phase A's only data-dependent computations are per-subject thresholds and weights, both from
  training-subject OOF rows.
- The aggregation rule, quantile values, arm definitions, and weight definitions are fixed by
  this document before Phase A runs.
- The seal is a byte-level commitment. Phase B recomputes it; a mismatch aborts the run.
- No output of Phase B may feed back into Phase A. A failure ends the experiment with a
  reported negative or inconclusive result.
- E1/E2 results, when available, do not enter Phase A or Phase B of E4 and do not modify any
  threshold, weight, arm, or selection rule (Section 5.2).

---

## 15. Reproducibility, outputs, compute

### 15.1 Proposed code (not created)

- `strict_loso_v1/experiments/E4/e4_common.py`
- `strict_loso_v1/experiments/E4/phase_a_thresholds.py`
- `strict_loso_v1/experiments/E4/phase_b_evaluate.py`
- `strict_loso_v1/experiments/E4/config_e4.json`

These must import the frozen functions from `strict_common.py` rather than copy or edit them.
`strict_common.py` is not to be modified.

**Dedicated output root (normative).** All E4 outputs are written under
`strict_loso_v1/experiments/E4/` (proposed; not created). That root is distinct from the E0 root
`strict_loso_v1`; no E4 file is written under `strict_loso_v1/folds/`,
`strict_loso_v1/results/`, or `strict_loso_v1/logs/`, and no E0 file is modified. The normative
roots are E0 `strict_loso_v1/`, E1 `strict_loso_v1/experiments/E1/`, E2
`strict_loso_v1/experiments/E2/`, E4 `strict_loso_v1/experiments/E4/`, and E5
`strict_loso_v1/experiments/E5/`. This path is a proposal and this document creates nothing.

### 15.2 Fixed constants

`seed = 42`; `k_sequences = 12`; `seq_sec = 30`; `max_gap_sec = 60`; `refractory_sec = 0`;
`far_auxiliary_occupancy_ceiling = 0.15`; `far_025 target = 0.25`; determinism from sorted
JSON and exact order statistics, no random tie-breaking anywhere.

### 15.3 Compute

| Phase | Work | Estimate |
|---|---|---|
| A | 23 OOF files (~1.5–1.9 MB each), 22 subjects × 9 ops × 23 folds threshold computations | 1–3 min CPU |
| B | 23 prediction files, 9 ops × 5 arms metric evaluation | 2–5 min CPU |
| Total | | well under 30 min CPU, < 2 GB RAM, no GPU |

The estimate assumes no refit. If a refit were required the experiment would no longer be
"reuse saved scores" and would need a new pre-registration.

### 15.4 Report contents

1. Threshold distribution per fold and arm (min, P25, median, P75, max, IQR).
2. Per-fold compliance table for E0-ref and A1–A4, with exact FAR and occupancy, plus the exact
   compliant-fold count `k/23` and `C_X` for every arm.
3. `DeltaC = C_A1 - C_E0` with sign and absolute value, and the improved / regressed / unchanged
   transition counts with fold identities.
4. Sensitivity guardrail computation, shown explicitly as counts out of 141, with the actual
   sensitivity and detected-seizure count; plus keys detected by E0 but missed by A1 and keys
   detected by A1 but missed by E0 where the artifacts support it.
5. All nine operating points for all five rows.
6. Pooled and macro metrics with intervals, plus per-subject tables.
7. Suppression review report: number and percentage of suppressing folds, boundary-contact
   folds, per-subject suppression, and, where supported, outer-test alerts and seizure
   detections in each suppressed fold.
8. The paired outer-fold bootstrap interval with the dependence limitation stated alongside it
   (never as a promotion verdict).
9. Provenance: protocol hash, E0 manifest verification result (46/46), recorded OOF and
   `training_meta.json` hashes, seal hash, dedicated E4 output root, reused-fold note, and the
   fixed `E0 -> E1 -> E2 -> E4 -> E5` execution-order reference.
10. The approved promotion criteria from Section 13.8 and Appendix B (U1, U2, U3), the
    descriptive-only role of the bootstrap, and the status of execution authorization (U6),
    repeated in the summary.

---

## 16. Risks and falsification

| Risk | Effect | Pre-registered handling |
|---|---|---|
| Noisy per-subject thresholds (subjects with 2 seizures, 15.6–154 h exposure) | Aggregated threshold unstable across folds | Report full distribution; do not trim outliers; A2/A4 show the conservative end |
| Inner-vs-final model score-scale shift | Achieved FAR/occupancy drift | Report achieved values; no rescaling; ambiguity acknowledged (Section 9) |
| Threshold suppression | Apparent compliance with no alarms | Strict suppression definition and counting (Section 13.7); >50% triggers a Mandatory Review Flag, not an automatic rejection |
| Non-monotone FAR under event merging | Threshold ordering counter-intuitive | Frozen exact union-find selection reused unchanged; flags reported |
| Weak baseline sensitivity (10/141) | Guardrail floor is 8/141 | Report as a non-inferiority floor, not clinical adequacy or equivalence; no sensitivity-improvement claim |
| The approved absolute floor (U1, 15/23) is already met by E0 | The absolute criterion alone cannot demonstrate improvement | U1 is retained as an independent floor; U2 (`DeltaC >= 2/23`, effective 19/23) supplies the improvement requirement; exact compliant counts always reported (Sections 13.3–13.4) |
| Phase A reads an outer-test-derived field or prediction content | Leakage into threshold construction | Field-level allowlist (Section 10.1); prediction `.npz` hashed only, never opened; violation is a stop condition |
| Promotion thresholds changed after inspecting E4 results | Results-driven rule | Both numerical requirements are PENDING before Phase B; changing them requires a new approved revision |
| G0 clauses conflict with the fixed sequence | Scheduling ambiguity | Reconciled in revision 4: roadmap clauses carry amendment notes, E5 follows E4, U4 closed (Sections 5.2, 5.4) |
| E4 or another experiment writes into the E0 root | Frozen E0 artifacts overwritten; comparisons invalid | Dedicated output root per experiment (Sections 1.5, 14, 15.1); path equality/nesting check is a stop condition |
| E0 manifest stores no OOF hashes | OOF inputs used without a manifest commitment | E4 records SHA-256 for every OOF file and `training_meta.json` and re-verifies before applying thresholds (Sections 5.1.9, 10.2, 11.1) |
| Reused `chb01` fold | Provenance heterogeneity | Documented; hash-compatible; reported as limitation |
| E0 reproduction mismatch | E4 thresholds would rest on unverified artifacts | Halt; no threshold applied; no E0 change |

Falsification conditions — E4 is falsified if any of the following holds:

1. A1 fails the approved absolute floor (`C_A1 < 15/23`) or the approved relative improvement
   requirement (`DeltaC < 2/23`, i.e. `C_A1 < 19/23` under the frozen baseline); the
   subject-robust hypothesis fails at the cohort level.
2. A1 compliance improves only through suppression-dominated folds. This triggers the mandatory
   review of Section 13.7 and is reported as a failure mode; it is not an automatic rejection
   before review.
3. A1 sensitivity falls below the `8/141` guardrail floor.
4. E0 reproduction fails, so no valid comparison exists.

---

## 17. Stop conditions

Stop and report, without applying any threshold:

1. Any precondition in Section 5.1 fails.
2. Any EDF-code reconstruction mismatch in Phase A or B (Section 4.7).
3. Any non-finite threshold, zero/negative weight, or failed equal-weight identity
   (Sections 8.2, 10.4).
4. Any Phase A allowlist violation (Section 10.1), including opening a
   `fold_*_predictions.npz`, reading its contents, or reading the `test` or `warnings` fields.
5. Any seal hash mismatch (Section 11.1).
6. Any outer-test integrity failure (Section 11.2).
7. Any E0 reproduction mismatch (Section 11.3).
8. Absence of the approvals required by Section 5.3. This is an authorization stop: no Phase A
   step may begin.
9. Any output-root overlap: the E4 output root equals, contains, or is nested inside the E0
   root, or any E0 source or artifact would be modified (Sections 1.5.8, 5.1.7, 14).
10. Any E0 manifest artifact hash mismatch (the 46 fold JSON and prediction `.npz` values), or
    any recorded OOF or `training_meta.json` hash differing between Phase A recording and seal
    verification (Sections 5.1.8–5.1.9, 10.2 A0b–A0c, 11.1).

In every case the fold or run is reported as failed with its reason. It is never dropped
silently, and it is never repaired by changing the protocol.

---

## Appendix A — Verification evidence (this revision)

All checks were read-only. No prediction score array (`fold_*_predictions.npz`,
`fold_*_predictions.csv`) was loaded or parsed, and no outer-test performance value was used
for any E4 decision. Inspected inputs were frozen aggregate tables, fold JSON metadata,
`folds/_status.json`, OOF artifacts, `training_meta.json`, and frozen source.

| # | Claim | Source inspected | Verified result |
|---|---|---|---|
| 1 | E0 run has 23 valid, hash-compatible folds | `folds/fold_*.json`, `manifest.json`, `folds/_status.json` | 23 JSONs, all `status=ok`, all hash `d39d831…ec8`, both npz present; `n_ok=22`, `n_failed=0`, `n_skipped=1` |
| 2 | `chb01` reused | `folds/_status.json`, `fold_chb01.json` | `status=skipped`, `runtime_sec=1335.391`, fold JSON `created_utc=2026-10-29T11:31:03Z` |
| 3 | E0 `far_025` = 10/141, 7.09%, FAR 0.242, occupancy 2.02% | `results/operating_points.csv` row `far_025` | counts 10/141; sens 0.070922; FAR 0.242097; occ 0.020184 |
| 4 | `C_E0 = 17/23` | `results/per_fold_operating_points.csv` | 17 folds satisfy FAR ≤ 0.25 and occ ≤ 0.05; 6 do not (`chb01, chb05, chb06, chb10, chb13, chb24`) |
| 5 | `chb01` values | `results/per_subject.csv`, `results/per_fold_operating_points.csv` | AUC raw 0.425389, smoothed 0.419559; `far_025`: 1/6, lead 1195, occ 0.149944, FAR 1.308120; `occ_05`: occ 0.308343 |
| 6 | `edf_code` is run-local | `strict_common.py:176-177` | `np.unique(..., return_inverse=True)` over 111,130 rows → 662 codes |
| 7 | Reconstruction reproduces frozen codes | 23 `fold_*_oof.npz` | 2,444,860 rows compared; zero mismatches |
| 8 | Multi-key rows exist | `training_meta.json`; `Auditing_V4.py:892-907`; `strict_common.py:179-205` | 38 rows with >1 id (30 with 2, 8 with 3); 3,616 links; 141 distinct keys |
| 9 | Detection is per key | `strict_common.py:940-950` | any fired row in a key's row list marks that key detected |
| 10 | 1,228 s is a start envelope | `training_meta.json`; `config.json:153-157` | 88 links start 2–28 s early; max start offset 28 s → 1,228; no positive end after `onset-300` |
| 11 | Smoothing 360 s / 330 s | `config.json:102-111`; `strict_common.py:407-443` | `k=12`, span `30×11=330`, included duration 360 |
| 12 | Suppression needs strict `>` | `strict_common.py:659-705`; `strict_loso.py:486` | threshold `nextafter(max,+inf)`; predicate `smoothed >= threshold` |
| 13 | Undefined metrics return NaN | `strict_common.py:547-555, 586-590, 915-917, 991-994` | confirmed |
| 14 | The earlier 14/23 absolute value was never ratified and is now withdrawn as a criterion | `FINAL_RESEARCH_ROADMAP.md` §6.4 (revision-3 text); repo-wide text search | roadmap formerly said "for example at least 60%"; the literal "14/23" appears in no pre-existing project file; the value is no longer a promotion threshold |
| 15 | G0 high-level order is now agreed; conflicting clauses are recorded, not silently rewritten | `FINAL_RESEARCH_ROADMAP.md` §6.4, §8.1, §8.2 (G0/G1), §8.3, §12.1; this revision | agreed order `E0 -> E1 -> E2 -> E4`; four pre-existing clauses conflict and are listed in Section 5.2 |
| 16 | Per-subject exposure | `results/per_subject.csv` | 23 subjects, 15.59–153.99 h, 896.33 h total; 141 seizures |
| 17 | `fold_subject_id` is not an OOF row identity | `fold_chb01_oof.npz`; `training_meta.json` | 106,462 OOF rows; 22 distinct `subject` values; `chb01` absent from its own OOF artifact |
| 18 | Fold JSON contains outer-test-derived fields that Phase A must not read | `fold_chb01.json` top-level keys | `test` and `warnings` are present; both excluded from the Phase A allowlist |
| 19 | `_status.json` contains per-fold outer-test warnings | `folds/_status.json` | per-fold `warnings` present; excluded from the Phase A allowlist |
| 20 | No E4 threshold-aggregation implementation exists | repo-wide search for aggregation/weighted-quantile code | none found; the Section 8.1 endpoint clamp is a specification, not a description of existing code |
| 21 | 2 pp guardrail arithmetic | frozen `10/141`; arithmetic | `10 - 0.02*141 = 7.18` -> at least 8; `8/141 = 5.674%`; `7/141 = 4.965%` fails |
| 22 | `DeltaC` bounds and count table | `C_E0 = 17/23`; arithmetic | `DeltaC` in `[-17/23, +6/23]`; counts 17/23 through 23/23 tabulated in Section 13.2 |
| 23 | All 46 fold-level E0 artifacts match the manifest | `manifest.json` `fold_results`; 23 `fold_*.json`; 23 `fold_*_predictions.npz` | 46/46 SHA-256 values match; `n_folds_ok = 23`, `n_folds_failed = 0`, `failures = []` |
| 24 | The E0 manifest contains no OOF hashes | `manifest.json` top-level and `fold_results[*]` keys | no `oof` key at either level; `fold_results[*]` = `{json, predictions_npz, runtime_sec, warnings}`; OOF hashing is therefore an E4-side recording requirement |
| 25 | No E4/E1/E2 implementation exists | repo-wide search; `strict_loso_v1/experiments` | no `experiments` directory and no E4/E1/E2 module; revision 4 creates none |
| 26 | E0 output-root separation is a real hazard | `strict_loso.py:922-943` | an existing fold is skipped only when `status == "ok"` **and** `protocol_hash` matches; a different config changes the protocol hash, marks E0 folds stale, and overwrites `fold_*.json`, `*_predictions.npz/csv`, `*_oof.npz`, and results in place under the same root |
| 27 | E0 files were unchanged by this documentation revision | before/after SHA-256 snapshot over 134 files under `strict_loso_v1` (`.py`, `.json`, `.csv`, `.md`, `.npz`) | identical before and after; no E0 source or artifact was written |
| 28 | E0 is complete across all 23 outer folds | `folds/_status.json`; `manifest.json`; 23 `fold_*.json` | 23/23 valid, protocol-hash compatible; `n_ok = 22`, `n_failed = 0`, `n_skipped = 1` (`chb01` reused); the one skipped fold is not a missing fold |
| 29 | E0 manifest lists exactly 46 fold artifacts | `manifest.json` `fold_results[*]` | 23 `fold_*.json` + 23 `fold_*_predictions.npz`; all 46 SHA-256 values match; `n_folds_ok = 23`, `n_folds_failed = 0`, `failures = []`; no OOF hash key |
| 30 | U1 and U2 arithmetic is internally consistent | this revision; frozen `C_E0 = 17/23` | U1 `15/23 = 65.22%`; U2 `2/23 = 0.0869565` (8.70 pp); effective `19/23 = 82.61%`; U1 emerges as a floor that E0 already meets |
| 31 | U3 arithmetic is unchanged and consistent | frozen `10/141`; arithmetic | `10 - 0.02 × 141 = 7.18` -> at least 8; `8/141 = 5.674%`; `7/141 = 4.965%` fails; non-inferiority, not clinical adequacy |

---

## Appendix B — Unresolved decisions register

This register is the authoritative list of E4 decisions. Resolved entries are retained with
their exact values for audit; unresolved entries state what decision is required, what is
already agreed, what remains undecided, and why the coding agent must not decide it silently.
As of 2026-11-02 only **U6 (execution authorization)** remains unresolved.

### U1 — Exact absolute compliance requirement for Promotion

- **Decision required:** the exact minimum number or fraction of the 23 outer folds that must
  satisfy `J_f = 1` under A1 for Promotion.
- **Decided 2026-11-02:** A1 must achieve at least **15 compliant outer folds out of 23**,
  i.e. `C_A1 >= 15/23 = 0.6522 = 65.22%`. A fold is compliant only if `FAR <= 0.25 alarms/hour`
  **and** `occupancy <= 0.05` at `far_025`. U1 is an independent absolute minimum floor.
- **Relationship to E0 and U2:** `C_E0 = 17/23 = 73.91%`, so 15/23 is already met by E0; U1 is a
  non-inferiority floor and cannot by itself demonstrate improvement. U2 requires at least
  19/23 under the current baseline, so passing both effectively requires at least 19/23. U1 is
  not removed because U2 is currently stricter.
- **Historical value:** the previously drafted `14/23` is withdrawn and is not active.
- **Status:** **APPROVED 2026-11-02 (resolved).** Approval does not authorize E4 execution.

### U2 — Exact relative improvement requirement for Promotion and its rationale

- **Decision required:** the exact minimum `DeltaC = C_A1 - C_E0` (or an equivalent relative
  statement) required for Promotion, together with a written rationale.
- **Decided 2026-11-02:** A1 must improve compliance by at least **two outer folds** relative
  to E0: `DeltaC >= 2/23 = 0.0869565 = 8.70 percentage points`. With
  `C_E0 = 17/23 = 73.91%`, this requires `C_A1 >= 19/23 = 82.61%`. `DeltaC` is a difference in
  fold-compliance proportions, not a percentage change relative to the E0 value.
- **Rationale:** a two-fold improvement is the pre-specified operational minimum for showing
  that subject-robust aggregation improves on the frozen baseline rather than merely matching
  it. It is not presented as statistically significant and is not proof of generalization.
- **Historical value:** the fixed `0.20` cutoff is withdrawn and is not active. The approved
  `2/23` is a separate, explicitly approved criterion, not a reinstatement of `0.20`.
- **Reporting:** `DeltaC` is always reported with its sign and absolute value, together with
  exact compliant-fold counts and improved/regressed/unchanged transitions.
- **Status:** **APPROVED 2026-11-02 (resolved).** Approval does not authorize E4 execution.

### U3 — Confirmation of the 2-percentage-point sensitivity margin

- **Decision required:** formal confirmation of the 2-percentage-point non-inferiority margin
  (`Sensitivity_A1 >= Sensitivity_E0 - 0.02`, i.e. at least `8/141` at `far_025`) and its role.
- **Decided 2026-11-02:** the 2 pp absolute margin is **approved**. The floor is at least
  `8/141 = 5.674%`; `7/141 = 4.965%` fails. Its role is a **sensitivity non-inferiority
  guardrail** applied to A1 at `far_025`. It is not a clinical-adequacy claim, not an
  equivalence test, and not statistically derived from E0. Actual sensitivity and detected
  counts are always reported, together with E0-only and A1-only detection differences where the
  artifacts support them.
- **Why the decision mattered:** changing the margin after results would be post hoc, and
  passing the guardrail must not be described as preserved or adequate sensitivity.
- **Status:** **APPROVED 2026-11-02 (resolved).** No promotion and no E4 execution is implied.

### U4 — G0 execution-order and dependency reconciliation

- **Decision required:** confirm that the fixed sequence `E0 -> E1 -> E2 -> E4` supersedes the
  conditional reordering and concurrency clauses of the roadmap, and state how the conflicting
  G0/G1 language is to be read.
- **Decided 2026-11-01 and completed 2026-11-02:** the fixed order is `E0 -> E1 -> E2 -> E4 ->
  E5`; E1 and E2 are completed before E4; E4 remains an independent frozen-score experiment;
  E1/E2 results must not modify E0 scores, E4 thresholds, aggregation weights, arm definitions,
  or selection rules; E4 is not tuned after inspecting E1/E2 findings; E5 follows E4 and its
  baseline is fixed by the pre-registered E4 promotion outcome. In revision 4 the roadmap
  clauses in §6.4, §8.1, §8.2 (G0/G1), and §8.3 carry amendment notes; the historical wording
  is preserved but is not active.
- **Status:** **RESOLVED 2026-11-02.** No execution authorization follows.

### U5 — Occupancy moving-block bootstrap role

- **Decision required:** whether the optional 12-sequence moving-block bootstrap on occupancy is
  reported, and in what role.
- **Decided 2026-11-02:** the occupancy moving-block bootstrap is **NOT PART OF E4**. It is not
  computed, not reported, and not referenced as an E4 result. Occupancy remains a point
  estimate with fold-level values.
- **Why the decision matters:** a temporal-block interval invites an over-stated
  temporal-stability claim; excluding it keeps the E4 uncertainty statement limited to the
  frozen E0 methods and the descriptive paired outer-fold bootstrap.
- **Status:** **RESOLVED 2026-11-02 — excluded.**

### U6 — Overall E4 execution authorization

- **Decision required:** explicit authorization to implement and execute E4.
- **Already agreed:** the promotion rule (U1–U3), execution order (U4), and exclusion of the
  occupancy moving-block bootstrap (U5) are resolved. No implementation or execution step is
  authorized; E4 remains PROPOSED / NOT APPROVED / NOT IMPLEMENTED / NOT EXECUTED.
- **Remains undecided:** authorization. U1 and U2 are now approved, but that approval does not
  itself authorize execution; explicit U6 approval is still required.
- **Why it must not be decided silently:** running E4 would spend the single outer-test read
  before execution is authorized. Approving the promotion rule fixes what a result would mean;
  it does not approve spending that read.
- **Status:** **NOT APPROVED.**

### Decided in revision 5 (not unresolved)

- U1 is **approved**: `C_A1 >= 15/23 = 65.22%`, with fold compliance requiring both
  `FAR <= 0.25/h` and `occupancy <= 0.05` (Section 13.3; U1).
- U2 is **approved**: `DeltaC >= 2/23 = 0.0869565` (8.70 pp), effective `C_A1 >= 19/23 =
  82.61%` under the frozen E0 baseline; U1 remains an independent floor (Section 13.4; U2).
- U3, U4, and U5 are unchanged from revision 4: 2 pp / `8/141`; `E0 -> E1 -> E2 -> E4 -> E5`;
  occupancy moving-block bootstrap excluded (Sections 13.5, 5.2–5.4, 12).
- U6 remains **NOT APPROVED**; the approved promotion rule does not authorize execution
  (Section 5.3; U6).
- E0 is recorded as **complete — 23/23 valid folds** (22 newly computed + one reused `chb01`,
  zero failed folds) in the reconciled project documents (Section 1.4 D22).
- The normative output roots are E0 `strict_loso_v1/`, E1 `strict_loso_v1/experiments/E1/`,
  E2 `strict_loso_v1/experiments/E2/`, E4 `strict_loso_v1/experiments/E4/`, and E5
  `strict_loso_v1/experiments/E5/` (Section 1.4 D23).

### Decided in revision 4 (not unresolved)

- The arm roster is exactly A1–A4; conformal and maximum-threshold alternatives are superseded
  and inactive (Section 6).
- The occupancy moving-block bootstrap is **NOT PART OF E4** (Section 12; U5).
- The 2-percentage-point sensitivity margin is **approved** as a non-inferiority guardrail
  (Section 13.5; U3).
- The bootstrap procedure has a single name: **paired outer-fold bootstrap (descriptive)**
  (Section 13.6).
- Phase A's `_status.json` integrity fields and manifest permissions are enumerated, and no
  precondition depends on a forbidden runtime or warning field (Section 10.1).
- Every experiment uses a dedicated output root; E4 reads E0 without modifying it (Sections
  1.5, 14, 15.1).
- The 46 E0 fold-level artifacts are verified against the manifest, and E4 records OOF and
  metadata hashes because the manifest stores none (Sections 5.1, 10.2, 11.1).
- The roadmap and companion documents are reconciled to the fixed order and approved criteria;
  E5 follows E4 (Sections 5.2, 5.4).

### Decided in revision 3 (not unresolved)

- The paired outer-fold bootstrap is **descriptive only**, not a promotion gate (Section 13.6).
- More than 50% suppressed folds is a **Mandatory Review Flag**, not a Promotion Gate
  (Section 13.7).
- `DeltaC` is retained and reported; the fixed 0.20 relative cutoff is withdrawn (Sections
  13.2–13.4).
- The sensitivity allowance is narrowed from 5 to 2 percentage points, giving `8/141`
  (Section 13.5); the 2 pp margin was formally approved in revision 4.
- The high-level execution order is `E0 -> E1 -> E2 -> E4` (Section 5.2; roadmap amendment).
- The canonical EDF identity is `(subject_id, edf_filename)`; `(fold_id, edf_code)` is
  artifact-scoped only (Section 4.7).
- Phase A has a field-level allowlist (Section 10.1).
- The weighted-quantile endpoint clamp is part of the specification (Section 8.1).

Previously resolved and still not blocking E4:

- **1,228 s.** Verified as a positive-sequence start-offset envelope, not lead support; excluded
  from the E4 specification and from every E4 decision (Section 4.5).
- **Multi-key rows.** Verified against metadata and evaluator; no reconciliation gap; does not
  block E4 (Section 4.6).
- **`edf_code` reconstruction.** Verified deterministic across all 23 OOF artifacts
  (Section 4.7).
- **E0 reuse.** Accepted with documented provenance; `n_skipped = 0` is not required
  (Section 5.1).

---

## Change log — revision 5 (2026-11-02 promotion criteria approved)

| # | Change | Location |
|---|---|---|
| R5-1 | Document renumbered E4-PREREG-005; decision register extended with D18–D23; supersedes revision 4 | Header; Section 1.4 |
| R5-2 | U1 **approved**: `C_A1 >= 15/23 = 65.22%`; fold compliance requires `FAR <= 0.25/h` and `occupancy <= 0.05`; independent absolute floor | Section 13.3; Appendix B U1 |
| R5-3 | U2 **approved**: `DeltaC >= 2/23 = 0.0869565` (8.70 pp), effective `C_A1 >= 19/23 = 82.61%`; U1 retained; prespecified operational criterion, not a significance claim | Section 13.4; Appendix B U2 |
| R5-4 | Count table extended to 15/23 and 16/23 with the approved U1/U2 promotion status; withdrawn 60%, 5 pp, 14/23, and 0.20 values remain inactive | Section 13.2 |
| R5-5 | Promotion section, status summary, default behaviour, risks, and falsification conditions updated to the approved rule; U6 remains not approved | Sections 13, 16, 13.8 |
| R5-6 | U3 (2 pp / `8/141`), U4 (fixed order), and U5 (moving-block excluded) preserved unchanged | Sections 13.5, 5.2–5.4, 12; Appendix B U3–U5 |
| R5-7 | E0 completion status reconciled across `PROPOSED_EXPERIMENTS.md`, `EXPERIMENT_PRIORITY_MATRIX.csv`, `PROJECT_STATE_AUDIT.md`, `FINAL_RESEARCH_ROADMAP.md`, and `UNRESOLVED_ISSUES.md`; historical 1/23 statements retained only as dated history | Section 1.4 D22; companion documents |
| R5-8 | Normative output roots fixed: E0 `strict_loso_v1/`, E1 `.../experiments/E1/`, E2 `.../experiments/E2/`, E4 `.../experiments/E4/`, E5 `.../experiments/E5/` | Sections 1.4 D23, 15.1; companion documents |
| R5-9 | Verification evidence rows 28–31 added; statement of non-execution updated; status remains PROPOSED / NOT APPROVED / NOT IMPLEMENTED / NOT EXECUTED | Appendices A and B; statement of non-execution |

---

## Change log — revision 4 (2026-11-02 decisions)

| # | Change | Location |
|---|---|---|
| R4-1 | Document renumbered E4-PREREG-004; decision register extended with D9–D17 | Header; Section 1.4 |
| R4-2 | Arm roster fixed at exactly A1–A4; conformal and maximum-threshold arms declared superseded and inactive in every document | Section 6 |
| R4-3 | Occupancy moving-block bootstrap declared **NOT PART OF E4**; optional wording removed | Section 12; Appendix B U5 |
| R4-4 | 2-percentage-point sensitivity margin **approved** as a non-inferiority guardrail (at least `8/141`); non-adequacy qualification retained | Section 13.5, 13.8; Appendix B U3 |
| R4-5 | Single bootstrap name enforced: **paired outer-fold bootstrap (descriptive)**; synonym removed | Section 13.6 |
| R4-6 | Phase A allowlist precision: `n_ok`/`n_failed`/`n_skipped` permitted; manifest and `fold_results[*]` fields enumerated; runtime and warning fields forbidden; precondition-verifiability statement added | Section 10.1 |
| R4-7 | Execution isolation: dedicated output root for every experiment; E4 output root normative; overlap is a stop condition | Sections 1.5, 5.1, 14, 15.1, 17 |
| R4-8 | E0 manifest integrity: 46/46 artifacts verified; OOF hashes recorded by E4 because the manifest has none; seal re-verification extended | Sections 5.1, 10.2, 11.1, 16 |
| R4-9 | Roadmap reconciliation completed; U4 closed; E5 placement recorded after E4 with its baseline fixed by the E4 promotion outcome | Section 5.2, 5.4; Appendix B U4 |
| R4-10 | Report contents, risks, and stop conditions updated for output-root isolation and manifest/OOF verification | Sections 15.4, 16, 17 |
| R4-11 | Verification evidence rows 23–27 added; unresolved register updated | Appendices A and B |
| R4-12 | U1 and U2 remain **PENDING USER APPROVAL**; no numeric replacement invented; document status unchanged (PROPOSED / NOT APPROVED / NOT IMPLEMENTED / NOT EXECUTED) | Sections 13.3–13.4, 13.8; Appendix B |

---

## Change log — revision 3 (2026-11-01 decisions)

| # | Change | Location |
|---|---|---|
| R3-1 | `DeltaC` retained as a primary operational comparison; fixed 0.20 relative cutoff withdrawn and not replaced; exact absolute and relative promotion requirements left **PENDING USER APPROVAL** | Sections 13.2–13.4, 13.8; Appendix B U1–U2 |
| R3-2 | Exact compliant-fold counts for E0-ref and every arm added, with the 17/23–23/23 count table and improved / regressed / unchanged transitions | Sections 13.2, 12 |
| R3-3 | `Paired subject bootstrap` renamed **paired outer-fold bootstrap (descriptive)**; explicitly never a gate; dependence limitation retained; never called an independent-subject bootstrap | Sections 13.6, 12; Appendix B |
| R3-4 | Sensitivity allowance narrowed from 5 to 2 percentage points; floor is `8/141 = 5.674%`; described as non-inferiority, not clinical adequacy; E0-only and A1-only detection differences added | Sections 13.5, 12, 15.4 |
| R3-5 | More than 50% suppressed folds changed to a **Mandatory Review Flag**, not a Promotion Gate; OOF suppression distinguished from outer-test false alarms; per-fold alert/detection reporting added | Sections 13.7, 13.8, 16 |
| R3-6 | G0 high-level order recorded as `E0 -> E1 -> E2 -> E4`; E4 independence and no-tuning rules stated; four conflicting roadmap clauses flagged; roadmap G0 section amended | Section 5.2; Appendix A #15; `FINAL_RESEARCH_ROADMAP.md` §8 |
| R3-7 | EDF identity corrected: `fold_subject_id` is not an OOF row identity; canonical identity is `(subject_id, edf_filename)`; artifact-scoped `(fold_id, edf_code)` explained; verified 106,462 OOF rows / 22 subjects / held-out subject absent | Section 4.7; Appendix A #17 |
| R3-8 | Phase A replaced by a field-level allowlist; `test`/`warnings`/`counts`/`thresholds` excluded; prediction `.npz` hash-only and never opened; Phase B access only after seal verification | Section 10.1, 14, 17; Appendix A #18–#19 |
| R3-9 | Weighted-quantile endpoint clamp declared part of the normative algorithm; pseudocode and a numerical example added; 22-subject clamp non-effect stated | Sections 8.1, 8.2 |
| R3-10 | Unresolved register restructured to state required decision, agreed content, undecided content, and why it cannot be decided silently | Appendix B U1–U6 |

---

## Change log — revision 2

**Historical record.** Revision 2 established the on-disk pre-registration. Its operative
criteria were superseded by revision 3: the fixed 0.20 relative cutoff and the earlier
sensitivity floor are **not** part of this document's current rules (see the revision 3 change
log above and Section 13).

| # | Issue in the review | Resolution in this revision | Location |
|---|---|---|---|
| 1 | Promotion criterion ambiguity (absolute count vs relative cutoff vs paired bootstrap) | All three defined separately with exact arithmetic and provenance. None is presented as approved; all marked **PENDING USER APPROVAL**. Notes that (a) the absolute value derived from the roadmap's "at least 60%" example, (b) E0 already met it, so it cannot demonstrate improvement, and (c) the relative cutoff implied 22/23. **Superseded by revision 3: the absolute value is withdrawn and the relative cutoff is withdrawn without replacement.** | Sections 13.3, 13.4, 13.8; Appendix B U1–U2 |
| 2 | Sensitivity guardrail example was wrong (the old 1/141 hypothetical) | Recomputed from the verified E0 baseline: 10/141 → floor 2.95 detections → at least 3 detections (2.13%); 2 detections (1.42%) fail. The hypothetical example was removed. **Superseded by revision 3: 2 pp allowance / at least 8 of 141.** | Section 13.5 |
| 3 | Section 2.1 E0 values contradicted Section 3.3 | All E0 values re-verified. `chb01` corrected: at `far_025`, occupancy 14.99% and FAR 1.308/h (30.83% belongs to `occ_05`; 1.82/h belongs to higher points). Section 3.3 rewritten to state that E4 generates no E0 results, uses frozen E0 results as reference, and quotes only historical values | Sections 2.1, 3.3 |
| 4 | `edf_code` reconstruction must be deterministic | Exact reconstruction (`np.unique` over the full `training_meta.json` array, exactly as the frozen loader does) specified; per-subject/fold regeneration and bare-code joins forbidden; canonical identity `(subject, edf_filename)`; verification against stored codes made mandatory before EDF-dependent operations; mismatch blocks the operation. Verified on 2,444,860 rows across 23 files | Section 4.7; Appendix A #7 |
| 5 | Overlapping seizure-key representation | Verified that 38 positive rows carry 2–3 identifiers, that the catalog places such a row in each linked key's row list, and that detection is per key, so one alerting row can count for several keys. Documented as inherited E0 semantics, unchanged in E4; merging affects labels only | Section 4.6; Appendix A #8–#9 |
| 6 | Paired bootstrap status | Retained, with the dependence limitation stated explicitly (folds share 21/22 training subjects; bootstrap is not a sample of independent populations; interval is not evidence of external generalization). The gate-versus-descriptive role was marked pending in revision 2. **Superseded by revision 3: descriptive only, not a gate.** | Section 13.6; Appendix B U2 |
| 7 | Preserve valid design decisions | Preserved: saved-score aggregation with no retraining; `far_025` as the only deciding point; joint FAR/occupancy compliance with no tolerance; E0-ref + A1–A4 with only A1 promotable; 23 valid artifacts / zero failures / documented reuse (no `n_skipped = 0`); frozen eligibility catalog; 360 s included vs 330 s lookback; strict suppression inequality; NaN semantics; Phase A/B separation, hashing, sealing, E0 reproduction; 900 s preictal vs 1,228 s start envelope. Revision 2 recorded G0 as unresolved. **Superseded by revision 3: G0 order is `E0 -> E1 -> E2 -> E4`, with conflicts recorded.** | Sections 4, 5, 6, 10, 11, 13 |

Also corrected or removed relative to earlier drafts:

- The 20% FAR/occupancy tolerance is removed everywhere; compliance uses the exact targets.
- The 90%-coverage conformal arm and maximum-threshold arm are removed.
- The Siena statement is removed as unrelated to E4.
- Section 13.5 no longer contains any non-baseline sensitivity example.

---

## Statement of non-execution

This document is a pre-registration proposal only.

- E4 has **not** been implemented, authorized, or executed. It is placed after E1 and E2 in the
  high-level sequence, but no execution date or authorization follows from that placement.
- Revision 5 is a documentation-only update. It modified this document,
  `FINAL_RESEARCH_ROADMAP.md`, `PROPOSED_EXPERIMENTS.md`, `EXPERIMENT_PRIORITY_MATRIX.csv`,
  `RESEARCH_IDEAS.md`, `UNRESOLVED_ISSUES.md`, and `PROJECT_STATE_AUDIT.md`; it created no
  directory, no experiment output root, and no code.
- No code file, directory, threshold, metric, or experiment output was created.
- No E0 artifact, configuration, label, feature array, or source file was modified.
- No model was trained, re-fitted, or re-run.
- No outer-test prediction score array was loaded, parsed, or evaluated.
- The only inspections performed were read-only: frozen protocol source, frozen E0 aggregate
  tables, fold JSON metadata (field names, not outer-test values used for decisions),
  `folds/_status.json`, OOF artifacts, and `training_meta.json`.
- The E0 artifact hashes recorded before and after this documentation revision are identical.

Execution requires explicit overall authorization (Appendix B U6). The promotion rule is now
fully approved: absolute floor U1 `C_A1 >= 15/23`, relative improvement U2 `DeltaC >= 2/23`
(effective `C_A1 >= 19/23`), and sensitivity guardrail U3 `>= 8/141`. The fixed execution order
(U4) and the exclusion of the occupancy moving-block bootstrap (U5) remain resolved. None of
those resolutions authorizes E4. Only after U6 is approved may the ordered
Phase A -> seal -> Phase B sequence described above begin.
