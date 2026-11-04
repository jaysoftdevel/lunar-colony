# Stage 1 — Controls and workflows (v0.50.14)

**One click selects, two quick clicks activate.** Double-click means two left clicks
on the same card (within 410 ms and 22 UI pixels). Dragging the research map
and clicks on a different item do not activate the previous card.

## Contract Office
- Single click a contract: open its project brief, deposit, final payout, research benefit, and complete research checklist.
- **Red MISSING** means research not unlocked; **green RESEARCHED** means completed. The checklist includes direct contract gates plus indirect prerequisites from the tech tree. The summary shows how many are complete.
- Use the checklist arrows at high zoom to inspect every requirement, including deep lunar-cargo prerequisites. A zero-prerequisite contract states that no research is needed.
- Double-click an available contract: sign it and collect its one-time deposit.
- Double-click a report-ready contract: deliver it and claim its final payout.
- Double-click an active contract: perform its current primary action (advance to next report).
- Completed contracts are permanent history and cannot be signed twice.
- The primary action button in the brief performs the same guarded operation.

## Research tree
- Single click a node: show cost, prerequisites and capability in the inspector.
- Double-click a researchable node: spend money and unlock that technology once.
- Locked nodes do not bypass prerequisites; unaffordable research is not purchased.
- Use the inspector's **UNLOCK TECHNOLOGY** button for the same action.
- Wheel zooms; drag empty graph background or right/middle drag to pan; FIT ALL and FOCUS assist navigation.

## Mission preparation
- The timeline shows DONE/CURRENT/LOCKED. Click CURRENT to open its facility.
- **GO TO NEXT FACILITY** opens the facility owning the current gate.
- Financial authorization and launch remain explicit buttons inside their
  facilities, so double-clicking a timeline tile cannot spend money or launch.
- Leaving for another menu does not reset mission choices, cargo or gate progress.

Navigation highlights the current section. NEXT STEP recommends where to go
without committing any resources.
