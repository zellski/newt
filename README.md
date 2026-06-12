## Introduction

This is NewT, an implementation of the Spacetime Constraints paradigm for the synthesis of realistic-looking motion. It is fully described in a thesis report, available at

    http://www.alyx.com/zell/thesis

where you probably got this code, too. All mathematical motivation is included in that report, and little of it will be repeated here.

## Summary of Dynamics System

In the current implementation, the virtual environment is very simple. The mechanics are planar, so bodies have zero depth. Animation subjects, "creatures", are constructed from rigid bodies, "limbs", that extend from the creature's root position in a tree topology. The system cannot handle cycles in the graph -- e.g. a model of a man with his hands clasped.

The time-varying state of a creature with N limbs can then be described in N+2 variables: two cartesians X and Y to describe where in the plane the root of the creature is, and N angles describing the attitude of each limb. These state-representing values are called degrees of freedom, "DOF", and as they change over a period of time, the creature moves. The goal of the paradigm is to create an animation, i.e. generate explicit descriptions of the DOF over time.

To do this, we represent the DOF over time as linear combinations (sums) of basis functions. Many useful bases exist, and a few are implemented here. To move, creatures also need muscles. The time-varying tension values of these are also represented explicitly in a function basis. A muscle sits at the joint between two limbs and produces a torque there.

The time-interval of the animation may be split into "stages". Over each stage, the DOF and their rates-of-change are continuous, so motion is smooth. Between stages, impact may occur, which has the effect of producing angular velocity discontinuities. In other words, when a creature slams into e.g. a floor, there is an infinite force applied to the creature for an infinitesimal period of time. This singularity requires special handling, and is the main motivation behind the introduction of stages.

This is a 30-line summary of a 50-page thesis. For understanding, go read the full report.

## Installation

The code presented here absolutely relies on the HQP/Omuses optimization package bundle written by R�diger Franke. It is available at

    http://hqp.sourceforge.net/

and is distributed under the Gnu Library General Public License. Compile and install this package.

- - -

Next, type 'make' in the newt/src subdirectory. If all goes well, the code will compile and turn into a library called libnewt.so. If not, you'll have to tinker a bit with src/Makefile.

Finally, 'make' in the newt/demo subdirectory. This builds the solver binary 'odc', which is driven through a Tcl program called 'run'. Yes, a Tcl installation is needed, but if you've installed HQP as per above, you already know that. The system sqlite3 library is also linked (present out of the box on macOS and most Linuxes); the YAML parser (fkYAML) is vendored under include/vendor.


## Tests

The newt/tests subdirectory holds standalone unit tests for the math
underpinnings: the AVec vector algebra, the numerical integration
rules, and the dynamics distribution machinery (force/impulse
application and the recursive sweeps onto angular DOFs). They compile
the real sources against a stub adouble instead of HQP/ADOL-C, so
they run anywhere:

    make -C tests test

## Running It

In the newt/demo subdirectory, typing e.g.

    ./run ../scenarios/luxo.yaml

solves the Luxo scenario. An optional second argument caps the SQP
iteration count. Every run gets a private scratch directory under
newt/runs/ (ADOL-C drops fixed-name tape files into the cwd, so
concurrent runs need the isolation -- and with it, you can launch
several runs at once, e.g. for multi-start experiments).

Results land in a per-project SQLite database, newt.db at the repo
root (override with the NEWT_DB environment variable). One row per
run, one row per accepted SQP iterate: objective, infeasibility, the
raw optimizer vector x, and the DOF trajectories sampled on a
record_dt grid -- so the whole history of how a motion converged is
queryable:

    sqlite3 newt.db "SELECT id, scenario_name, status, final_iter FROM runs"
    sqlite3 newt.db "SELECT k, objective, infeasibility FROM iterations WHERE run_id = 3"

Trajectory frames and x live as little-endian float64 blobs (times,
frames, x columns); dof_names and stage_bounds are JSON. The scenario
and creature YAML are snapshotted into the runs row, so a database is
self-describing. Multiple processes can write concurrently (WAL mode),
and readers never block -- a viewer can follow a run live.

## Scenario Files

Scenarios live in newt/scenarios as two YAML files: a creature (the
body plan) and a scenario (everything else), referencing the creature
by relative path. The five thesis demos there double as schema
documentation; in brief:

A creature file has `name`, `root: {x, y, attach}` (the anchor's two
positional DOF names and the Body.Point it grabs), `bodies` (a
sequence -- `type: sphere|rod|disk|cylinder`, a `dof` name, dimensions
`radius`/`length`/`height` and `density` as the type requires, and
named `points` in body coordinates), and `attachments` (parent/child
Body.Point pairs forming the tree).

A scenario file has `name`, `creature`, `gravity`, `integrator:
{type: simpson|gauss, n}`, optional `record_dt` (default 0.01), and
`stages`. Each stage: `pieces`, `duration` (a number, or `{min, max,
start}` to make the duration an optimizer variable), `muscles`
(`{dof, weight}`), `reps` (exactly one per DOF: `{dof, type:
constant|hermite|hermlet|hat, value | from/to [, min/max]}`),
plus optional nested `impulses` (`{point, direction: [x, y]}`, add
`magnitude` to fix it instead of letting the optimizer choose) and
`constraints` (`{dof, quantity: q|qdot, at: start|end|{slice, t},
equals | min/max}`). Top-level `impulses` and `constraints` (with a
`stage` key) and `links: [[A, B], ...]` (inter-stage continuity +
impulse handling) round it off.

One sharp edge worth knowing: construction order follows document
order, and optimizer variable indices follow construction order. The
shipped scenarios reproduce their C++ ancestors' iteration tables
bit-for-bit because their collections are listed in the original
construction order. Reordering entries changes nothing mathematically
but can perturb the optimizer's path.

Validation is strict -- unknown keys, missing DOF representations,
dangling references and non-power-of-two Hermlet stages are all
rejected with a path to the offending entry.

## Roll Your Own

The scenarios in newt/scenarios should explain how to go about
configuring your own. Unfortunately, there are numerous disclaimers
and caveats in the current implementation. The whole optimization
business is tricky to say the least; sometimes the process feels more
like black magic than science. I suggest slowly modifying one of the
examples in a direction you wish to go.

A few hints:

* Variable bounds are -essential- for keeping the optimization process on a sane track. Without bounds, the search-space is just too large and too chaotic. At the very least, add bounds that you -know- won't be broken, such as e.g. -10PI < AngularDOF < 10PI.

* The initial values are likewise essential. These are the from/to fields of a representation, and currently only linear interpolation between them is supported. The initial values determine where in the search-space the optimization process starts, which in turn entirely determines how it perceives the search space. Different initial values will very often lead to different solutions. Remember that there is no such thing as practical -global- optimization.

* In most cases, keeping stages short helps convergence; alternatively, turn off gravity while experimenting. Long stages and high gravity makes for a chaotic search-space. Short stages removes potential fancy motions from consideration.

* Experiment using Hat as well as Hermlet representations for Muscles. Using Hat means clumsier control for the Creature, and the motion may look odd. On the other hand, convergence sometimes benefits greatly from this relationship between the DOF and Muscles (Assuming Hermites are still used for the DOF!) and so N can be increased the compensate.

## Rendering

The original RenderMan/Octave file output is retired; everything a
visualizer needs now lives in the run database. A web-based viewer
consuming it -- with scrubbers for both animation time and optimizer
iteration -- is planned (see GitHub issue #3). The old RIBVisualizer
source remains in the tree but is no longer invoked.

Pär Winzell
zell@alyx.com
