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

The code presented here absolutely relies on the HQP/Omuses optimization package bundle written by Rüdiger Franke. It is available at

    http://hqp.sourceforge.net/

and is distributed under the Gnu Library General Public License. Compile and install this package.

- - -

Next, type 'make' in the newt/src subdirectory. If all goes well, the code will compile and turn into a library called libnewt.so. If not, you'll have to tinker a bit with src/Makefile.

Finally, 'make' in the newt/demo subdirectory. This will compile a number of example creatures/scenarios into a binary called 'odc' which is then run through a TCL program called 'run'. Yes, a TCL installation is needed, but if you've installed HQP as per above, you already know that.

- - -

The code outputs RenderMan format and you will need a renderer for it. For a summary of such programs, see:

    http://www.faqs.org/faqs/graphics/renderman-faq/


## Running It

In the newt/demo subdirectory, typing e.g.

    ./run Luxo 2>debugout

will execute the Luxo module properly, through Tcl.

Output ends up in the newt/res subdirectory. Several files are written there each iteration and can be investigated to visualize the optimization process. The RenderMan output ends up in the file

	newt/res/snapshot.dat

and using e.g. the Blue Moon Rendering Tools (BMRT), you display it with e.g.

	rgl snapshot.dat

The other files are on the pattern

	qAlphaDat.m		    DOF values over the interval
	qAlphaDotDat.m		DOF rate of change
	QAlphaDat.m		    Muscle values over the interval
	qCAlphaDat.m		Inertial terms associated with this DOF
	qMAlphaDat.m		Generalized momentum relative to this DOF

for a DOF named "Alpha"; other files are written for other DOF.

The first three can be quite useful for visualisation. The format is constructed to work with Gnu Octave. It is quite possible they also work with Matlab.

## Roll Your Own

Basically, the examples in the newt/demo subdirectory should explain how togo about configuring your own scenarios. Unfortunately, there are numerous disclaimers and caveats in the current implementation. The whole optimization business is tricky to say the least; sometimes the process feels more like black magic than science. I suggest slowly modifying one of the examples in a direction you wish to go and emailing me if there are difficulties.

A few hints:

* Variable bounds are -essential- for keeping the optimization process on a sane track. Without bounds, the search-space is just too large and too chaotic. At the very least, add bounds that you -know- won't be broken, such as e.g. -10PI < AngularDOF < 10PI.

* The initial values are likewise essential. These are supplied to the constructor of e.g. Hermite() and currently only linear interpolation is supported. The initial values determine where in the search-space the optimization process starts, which in turn entirely determines how it perceives the search space. Different initial values will very often lead to different solutions. Remember that there is no such thing as practical -global- optimization.

* In most cases, keeping stages short helps convergence; alternatively, turn off gravity while experimenting. Long stages and high gravity makes for a chaotic search-space. Short stages removes potential fancy motions from consideration.

* Experiment using Hat as well as Hermlet representations for Muscles. Using Hat means clumsier control for the Creature, and the motion may look odd. On the other hand, convergence sometimes benefits greatly from this relationship between the DOF and Muscles (Assuming Hermites are still used for the DOF!) and so N can be increased the compensate.

## Rendering

I have included a subdirectory newt/res/RENDER in the distribution, with a makefile in it that assumes the existance of the BMRT renderer as well as the 'transcode' utility:

    http://www.theorie.physik.uni-goettingen.de/~ostreich/transcode/

By default, it generates a MPEG4 stream, which most modern computers can view and which has excellent compression.

Pär Winzell
zell@alyx.com
