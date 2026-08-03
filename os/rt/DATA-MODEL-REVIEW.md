# RT Data Model and Structures — Correctness Review

**Status:** second cross-check complete. Eight defects are confirmed: DM-1
through DM-4 and DM-8 through DM-11. DM-11 is documentation-only.
DM-5 and DM-6 are qualified layout-hardening items, while DM-7, DM-12, and
DM-13 are not defects. The status table below is authoritative; stable IDs are
retained even when a proposed finding is rejected.

| | |
|---|---|
| Date | 2026-08-03 |
| Branch / commit | `chibios-kernel-dev` @ `6882bcf366f5` |
| Main scope | `thread_t`, registry and reference lifecycle, intrusive containers, virtual timers, `os_instance_t`, `ch_system_t`, debugger layout metadata |
| Related report | `MUTEX-REVIEW.md` covers mutex ownership and priority inheritance in detail; those findings are not duplicated here |
| Build used | `test/rt/testbuild`, SIMX86_64 |
| Independent cross-check | `DATA-MODEL-CROSSCHECK.md`, anchored to `master` @ `fc0eb9e15b` |

## PR classes

The classes are proposed change boundaries. Each class can be reviewed and
merged independently.

1. **Class A — thread identity and lifecycle invariants**

   Defects: DM-1, DM-2, and DM-8.

   Scope: make registry lookup compatible with nullable names, enforce the
   terminal-state requirement when disposing externally stored thread objects,
   define and document the reference-count bound, and add focused registry and
   thread-lifecycle tests.

2. **Class B — debugger layout representation**

   Defect: DM-3.

   Scope: prevent silent narrowing in `ch_debug`, either by versioning and
   widening the layout record or by rejecting unrepresentable configurations at
   compile time.

3. **Class C — virtual-timer object state**

   Defect: DM-4.

   Scope: make the dispose-time integrity check understand the virtual timer's
   disarmed sentinel and run the timer lifecycle tests at hardening level one.

4. **Class D — intrusive-layout contracts**

   Items: qualified DM-5 and DM-6, plus documentation defect DM-11.

   Scope: use the state-correct union member in the remaining PI requeue path,
   document and compile-time-check the `thread_t.hdr` overlap assumptions, and
   replace the false priority-queue pointer note. No new generic assertion
   facility is required; `_Static_assert` and standard `offsetof()` already
   have a project precedent.

5. **Class E — round-robin configuration validation**

   Defect: DM-9.

   Scope: reject negative and unrepresentable `CH_CFG_TIME_QUANTUM` values and
   add boundary build tests.

6. **Class F — memory-area type unification**

   Defect: DM-10.

   Scope: make `MEM_AREA_DECL()` produce the `memory_area_t` consumed by oslib,
   preserving a compatibility alias if the transitional public type cannot be
   removed immediately.

Recommended order: C, A, D, E, F, then B. Class C must land before the full
hardening-level lifecycle sweep can run. The other classes are independent.

## Confirmed defects

Disposition of the independently proposed additions:

| ID | Converged classification | Reason |
|---|---|---|
| DM-5 | Qualified portability/maintenance item | State-inappropriate union access violates the typed model but works with the current offset-zero layouts; this retains the classification already converged in `MUTEX-REVIEW.md` |
| DM-6 | Qualified invariant-hardening item | The overlap property is load-bearing and should be asserted, but it holds on the reviewed implementation; a deliberately reordered mutation is not a current failure |
| DM-7 | Not a defect | Direct `_Static_assert` and standard `offsetof()` already have project precedent in the RISC-V Hazard3 port, so no new facility is required |
| DM-8 | Confirmed contract/bounds defect | The public reference limit is undocumented and three increment paths embed the type limit as a literal |
| DM-9 | Confirmed configuration defect, executed | A quantum of 256 compiles and is stored as zero |
| DM-10 | Confirmed public type mismatch, executed | `MEM_AREA_DECL()` produces a type rejected by the memory-area APIs included by the same `ch.h` |
| DM-11 | Confirmed documentation defect | Priority-queue links are typed pointers, not `void *` |
| DM-12 | Not a defect | The apparently inverted idle/main guard implements the intended no-separate-idle model correctly |
| DM-13 | Not a defect | The debugger fields retain semantic names: registry `next` is newer, `prev` is older, and `wabase` is used as the stack limit |

### DM-1 — registry name lookup dereferences nullable thread names

`thread_t.name` is explicitly documented as a thread name **or `NULL`**
(`chobjects.h:211-223`). `chThdObjectInit()` copies the descriptor name without
requiring it to be non-null (`chthreads.c:167-171`), and both registry name
setters can also store `NULL` (`chregistry.h:165-209`).

`chRegFindThreadByName()` nevertheless calls
`strcmp(chRegGetThreadNameX(ctp), name)` for every registered thread
(`chregistry.c:217-227`). A valid unnamed thread therefore makes lookup of any
name dereference a null pointer. A null search argument is not checked either.

The current registry test names its worker before performing lookup, so it does
not exercise the valid unnamed representation.

**Proposed fix:** require a non-null search key, skip registry entries whose
stored name is null, and add tests covering an unnamed descriptor followed by
both successful and unsuccessful named lookups. If a null search key is meant
to find unnamed threads instead, define that behavior explicitly and compare
without `strcmp()`.

### DM-2 — `chThdObjectDispose()` accepts live detached threads

The disposal contract says that the function verifies a state compatible with
stopping the object (`chthreads.c:189-203`), but the implementation checks only
auxiliary containers, references, and owned mutexes (`chthreads.c:204-230`). It
never checks `tp->state`.

The existing reference model proves that `refs == 0` is insufficient:
`chThdRelease()` deliberately allows a live thread to reach zero references and
calls it detached (`chthreads.c:645-687`). A detached READY, CURRENT, or blocked
thread can therefore satisfy all dispose checks. With hardening enabled,
`chThdObjectDispose()` then clears the live `thread_t` while the scheduler or a
wait queue still owns its shared intrusive header. Without hardening, the
function still reports successful disposal and permits the caller to reuse the
storage while it is live. When the registry is disabled there is not even a
reference-count check to partially mask the missing state invariant.

**Proposed fix:** assert that the object is in `CH_STATE_FINAL` before allowing
disposal. Retain the existing checks for waiters, messages, references, and
owned mutexes. Add negative lifecycle tests for detached READY and blocked
threads and a positive test after termination.

### DM-3 — debugger layout metadata silently truncates valid layouts

Every size and offset in `chdebug_t` is eight bits (`chregistry.h:42-87`). The
initializer explicitly casts `sizeof` and `offsetof` expressions to `uint8_t`
(`chregistry.c:76-139`) without a representability check. At the same time,
`CH_CFG_THREAD_EXTRA_FIELDS` is a documented extension point appended to
`thread_t` (`chobjects.h:364-367`) and has no size limit.

This was reproduced on SIMX86_64 by adding a 256-byte array through
`CH_CFG_THREAD_EXTRA_FIELDS`:

```
sizeof(thread_t)       = 432
ch_debug.threadsize    = 176
```

The configuration compiled and linked without a diagnostic. `432` narrowed
modulo 256 to the same `176` exported by the unextended build, so a debugger
cannot even detect the layout change from `threadsize`.

The cross-check reproduced the same defect with a different configuration:
statistics plus a 64-byte thread extension produced a 280-byte `thread_t` and
an exported size of 24. Tail extensions do not move the currently recorded
thread offsets, so `threadsize` is the immediately reachable configuration
failure. Offset checks are still appropriate because the port-defined context
precedes several recorded fields, and the record also narrows
`sizeof(struct port_intctx)`.

**Proposed fix:** the smallest compatible correction is to add compile-time
representability assertions for each encoded expression and document the
limits. A versioned record with wider fields is only needed if layouts above
those limits must be supported. Do not rely on explicit casts, because they
suppress the required diagnostic. Add a build-only configuration that
deliberately crosses the chosen boundary.

### DM-4 — virtual-timer disposal applies a circular-queue check to the null sentinel

The virtual timer data model uses `dlist.next == NULL` to mean disarmed:
`chVTObjectInit()` initializes only that member (`chvt.c:268-282`), reset paths
restore it to null, and `chVTIsArmedI()` tests it directly.

`chVTObjectDispose()` first calls `chSftCheckQueueX(&vtp->dlist)` and only then
checks that `dlist.next == NULL` (`chvt.c:284-310`). The queue checker expects a
circular, doubly linked queue and rejects null links. It also loads `prev`,
which `chVTObjectInit()` leaves indeterminate. It is therefore incompatible
with every correctly disarmed virtual timer.

This was reproduced by building and running the standard SIMX86_64 RT suite
with `CH_CFG_HARDENING_LEVEL=1`. Tests passed through sequence 12.8, then the
kernel stopped during teardown of test 12.9, "Virtual Timer object lifecycle",
when a reset/disarmed timer was disposed. The default suite uses hardening
level zero and cannot expose the mismatch.

The cross-check's stronger claim that an aligned garbage `prev` can make the
first pointer assertion pass is not correct: `next` is null, so the conjunction
always fails its first operand. The default halt hook prevents the following
back-link dereference. Loading an indeterminate `prev` is still unnecessary and
must be removed from this valid disposal path.

**Proposed fix:** use a timer-specific disposal check based only on the null
disarmed sentinel instead of `chSftCheckQueueX()`. If disposal at hardening
level one is also intended to reject armed timers when debug assertions are
disabled, make that state check a level-one safety assertion. Fully
initializing `prev` and `delta` would make the inactive representation
deterministic, but is not required once no disarmed path reads them. Add a
hardening-level-one test variant for object lifecycle tests.

### DM-8 — thread reference capacity is hidden and duplicated

`trefs_t` is an eight-bit counter, but its maximum is not exposed in the API
contract. `chThdAddRef()`, `chRegFirstThread()`, and `chRegNextThread()` each
repeat the literal 255 in a debug assertion before incrementing the counter
(`chthreads.c:635-640`, `chregistry.c:154-164`, and
`chregistry.c:180-199`).

Reaching the limit is not described as an invalid call. With assertions
disabled, another increment wraps 255 to zero. A final dynamic thread can then
be reclaimed by a later registry scan even though the application still holds
logical references accumulated before the wrap. On a live thread the same
wrap also makes the reference model temporarily identify it as detached.

The cross-check's statement that the immediately following
`chThdRelease()` frees the object is too direct: decrementing wrapped zero
produces 255. Reclamation instead occurs when final-state or registry-scan
logic observes the wrapped count as described above.

**Proposed fix:** define a `THREAD_MAX_REFERENCES` value derived from
`trefs_t`, document it as a precondition of every reference-producing API, and
use it at all three increment sites. The existing debug-assert policy can then
remain consistent with the mutex recursion limit; if release builds must
handle exhaustion, the API needs an explicit failure policy because it has no
error return.

### DM-9 — `CH_CFG_TIME_QUANTUM` accepts values outside `tslices_t`

`tslices_t` is `uint8_t`, while `chchecks.h:84-87` verifies only that
`CH_CFG_TIME_QUANTUM` is defined. Every assignment explicitly casts the option
to `tslices_t`, suppressing narrowing diagnostics.

This was independently reproduced with `CH_CFG_TIME_QUANTUM=256`. The standard
SIMX86_64 suite compiled and linked, and disassembly of `chThdObjectInit()`
showed a zero byte stored in `thread_t.ticks`. Scheduler reload paths likewise
store zero. Equal-priority threads are consequently treated as having
exhausted their quantum on every tick instead of receiving a 256-tick slice.
Values above 256 wrap modulo 256, and negative values silently select the
disabled compile-time path even though only zero is documented to disable
round robin.

**Proposed fix:** reject values below zero or above the maximum representable
`tslices_t` value in `chchecks.h`. Add accepted builds at zero, one, and the
maximum, plus rejected builds immediately outside the range.

### DM-10 — `MEM_AREA_DECL()` creates the wrong public memory-area type

`chalign.h:47-90` defines `memory_area_new_t`, `__MEM_AREA_DATA()`, and
`MEM_AREA_DECL()`. OSLIB independently defines the byte-identical
`memory_area_t`, and every real memory-area API consumes that second type
(`chmemchecks.h:46-61`). Both headers are included by the public RT `ch.h`.

A compile probe using `MEM_AREA_DECL(probe_area, NULL, 0)` and then passing
`&probe_area` to `chMemIsAreaWithinX()` fails with incompatible-pointer-type
diagnostics: the macro produces `memory_area_new_t *`, while the API requires
`memory_area_t *`. Tree-wide search finds no consumer of
`memory_area_new_t`; its history identifies it as part of unfinished "new API"
work.

**Proposed fix:** establish one definition of `memory_area_t` in the earliest
common header and make both RT and oslib use it. Preserve
`memory_area_new_t` as a typedef alias for a deprecation interval if external
source compatibility is required. `MEM_AREA_DECL()` must produce the type
accepted by the memory-area APIs without a cast.

### DM-11 — priority-queue documentation describes obsolete pointer types

The note at `chlists.h:79-88` says the link fields are `void` pointers to avoid
aliasing, but both are declared as `ch_priority_queue_t *`. This is factually
incorrect and obscures the actual overlay constraint in DM-6.

**Proposed fix:** replace the stale note with the real contract: `next` and
`prev` must occupy the same prefix used by `ch_queue_t`, while `prio` must not
overlap that prefix when the structures share `thread_t.hdr`.

## Qualified layout items

### DM-5 — PI requeue reads WTCOND and WTSEM through `u.wtmtxp`

The PI propagation switch in `chMtxLockS()` uses
`&tp->u.wtmtxp->queue` for `CH_STATE_WTCOND` and `CH_STATE_WTSEM`, although
those states store `u.wtobjp` and `u.wtsemp`, respectively
(`chmtx.c:245-258`). The newer `__thd_set_priority()` path already uses the
state-correct members (`chthreads.c:916-943`).

The current code works because all three pointed-to objects place
`ch_queue_t queue` at offset zero and supported object pointers share the
expected representation. No failing supported target has been identified, so
this remains the qualified portability/maintenance finding already recorded
as mutex finding 6, not a newly confirmed runtime defect.

**Recommended change:** split the combined switch arm and use the state-correct
member in each case, preferably through a shared internal helper.

### DM-6 — the shared thread header has an implicit overlap contract

`thread_t.hdr` overlays `ch_list_t`, `ch_queue_t`, and
`ch_priority_queue_t`. Priority must survive while the same header is linked
through the plain FIFO representation. The necessary property is:

```
offsetof(ch_priority_queue_t, prio) >= sizeof(ch_queue_t)
```

It holds on SIMX86_64 at equality, 16 bytes versus 16 bytes. The first-member
offset assumptions used by `threadref()` and the other intrusive containers
also hold. A mutation that reorders `prio` demonstrates why the contract
matters, but does not demonstrate a defect in the current source.

**Recommended change:** document the prefix/overlap rule and add direct
compile-time assertions for it and the offset-zero container assumptions.
There is no DM-7 blocker: the project already uses `_Static_assert` and
standard `offsetof()` for assembly-visible layouts in
`os/common/ports/RISCV-HAZARD3/chcore.c`. Migrating `__CH_OFFSETOF` to standard
`offsetof()` can be considered separately but is not required to state these
checks.

## Data-model invariants checked so far

- `thread_t.hdr` is one state-dependent intrusive membership shared by ready,
  wait, semaphore, mutex, condition-variable, and message queues. Registry
  membership correctly uses the separate `rqueue` member. Its priority field
  currently lies beyond the complete plain-queue prefix, so FIFO link writes do
  not overwrite priority; DM-6 recommends making that contract explicit.
- `thread_t.u` is state-dependent. `sentmsg` correctly remains outside it
  because priority message queues need `u.wtobjp` and the sent value at the
  same time.
- Mutex ownership has dedicated `mtxlist` and `realprio` fields; its invariants
  remain covered by `MUTEX-REVIEW.md`.
- Ready lists and virtual-timer lists are per `os_instance_t`; the registry and
  RFCU move to `ch_system_t` in SMP mode as their conditional layouts specify.
- Delta-list removal intentionally leaves the removed element's successor link
  available so the caller can repair the successor delta. Current reset and
  expiry paths rely on that contract consistently in same-instance operation.

## Open cross-checks — not yet findings

1. **Virtual-timer instance ownership in SMP.** `virtual_timer_t` has no owner
   field, while set, reset, remaining-time, and tick operations select
   `currcore->vtlist`. Cross-instance reset can unlink through the timer's own
   links but repair the current instance's header or alarm. The next pass must
   establish whether same-instance access is an intended but undocumented
   precondition or whether ownership must be represented and enforced.

2. **State-union transition audit.** Continue checking every timeout, reset,
   and wakeup path to ensure it removes `hdr` from its current container before
   overwriting `u`, particularly for suspend references, priority message
   queues, and event waits. DM-5 is the one state-inappropriate typed access
   identified so far; it is layout-compatible on the reviewed targets.

3. **Configuration matrix.** The independent cross-check expanded SIMX86_64
   coverage to statistics, full tracing, recursive mutexes, and an additional
   thread extension. Still repeat the layout checks on a small 8/16-bit target,
   a 32-bit Cortex-M target, SMP, and registry-disabled builds.

4. **Object lifecycle at hardening levels one through three.** The virtual
   timer failure shows that the default level-zero tests are not sufficient to
   validate dispose-time representations. Run all object lifecycle cases in
   each enabled layout.

## Verification record

- Default SIMX86_64 test build: compiled and linked successfully.
- Extended-thread experiment: compiled and linked; DWARF reported
  `sizeof(thread_t) == 432`, while the exported byte remained `176`.
- Hardening-level-one SIMX86_64 suite: reached test 12.9 and stopped in the
  virtual-timer dispose path as described in DM-4.
- `CH_CFG_TIME_QUANTUM=256` SIMX86_64 build: compiled and linked;
  `chThdObjectInit()` stores zero in the byte-sized `ticks` field.
- `MEM_AREA_DECL()` compile probe: rejected by `chMemIsAreaWithinX()` as an
  incompatible pointer type, confirming DM-10.
- Layout-hardening cross-check: current offset-zero and header-overlap
  properties hold on SIMX86_64. Existing Hazard3 sources confirm direct
  `_Static_assert`/`offsetof()` precedent, rejecting DM-7 as a blocker.
- `git diff 6882bcf366f5..fc0eb9e15b -- os/rt` is empty, so the independent
  report's newer commit anchor does not change any reviewed RT source.
- All temporary configuration changes were restored.
