![GPL licensed](https://img.shields.io/badge/license-GPL-blue.svg)

# BLACK HOLE LAB IS WORK IN PROGRESS
## Introduction

Black Hole Lab is a physics simulation of a rotating black hole and two
observers, Alice and Bob.  You can launch Alice and Bob on paths around or into
the black hole and watch events play out.  The simulation let's you see a slice
of the physics with one or two dimensions of space plus one of time.  Alice and
Bob also have a beacon that transmits a radio wave in all directions.  The
simulation calculates and shows the wavefront of these signals, accounting for
the warping of space and time by the black hole.  The simulation also records
whenever Alice and Bob receive a signal from the other.

The simulation takes pains to respect Einstein's equations of general
relativity.  To a double-floating point degree of accuracy, the effects shown in
Black Hole Lab are really what the math of general relativity says will happen.
While the predictions of general relativity have been proven correct countless
times, physicists agree that general relativity *cannot* be the final word on
black holes.  Nobody knows yet what would *actually* happen to Alice and Bob
inside a black hole!

## Starting Up

Black Hole Lab divides the screen into 3 parts.  On the left are most of the
settings.  In the middle is the selectable frame of reference (FoR) view or the
global "foliation" chart.  On the right is the 2 dimensional equatorial view.

![Starting Up](assets/images/startup.png)

## Basic Controls

To start, click **Play** in the top left.  You'll see Alice and Bob start moving
and broadcasting the their signal.  By default, Alice orbits the black hole
while Bob takes a rather uninteresting plunge inward.

![Basic Controls](assets/images/basic_controls.png)

Click **Reset** to return the simulation to its starting point.  You can also
play/pause the simulation with the space bar.  The keyboard arrow keys **single
step** the simulation forward and backward in time.

You can also save and load your simulation using the associated buttons.

## The Equatorial Plane View

The right hand side shows a top-down map of the black hole and surrounding
space.  The black hole looks like a colored bullseye while Alice and Bob are the
yellow and turquoise dots respectively.  The black hole spins counter-clockwise.

![Equatorial View](assets/images/equatorial_view.png)

### The Black Hole Ergosphere

The outermost yellow region of the black hole is the *ergosphere*.  A spinning
black hole pulls the very vacuum of space around in the direction of spin, a
phenomenon known as **frame dragging**.  Frame dragging affects space out to
great distances, but the ergosphere marks the boundary where dragging pulls
space *around* the black hole faster than the speed of light!  Objects can enter
and escape the ergosphere, but *nothing* can stay motionless there.

More details:
[Wikipedia](https://en.wikipedia.org/wiki/Ergosphere),
[Anton Petrov](https://www.youtube.com/watch?v=crnxcK2UazM),
[PBS Space Time](https://www.youtube.com/watch?v=UjgGdGzDFiM)

### The Black Hole Event Horizon (Region II)

Next inside the ergosphere is the light blue *event horizon*.  The event horizon
marks the boundary where the vacuum of space moves *into* the black hole faster
than the speed of light.  This is the point of no return!  An object can cross
the event horizon going in, but cannot cross back into *our* universe.  Crossing
the event horizon puts you in what physicists call `region II` of the black
hole.

Note that general relativity says spinning black holes have *two* event
horizons.  The horizon we're discussing here is the *outer horizon* that
physicists give the shorthand name `r+`.

To put it mildly, interesting things happen at the outer event horizon.  For
example, immediately after crossing, the event horizon stops being a place below
you and becomes a moment in your past!  Inside, time itself (your future) points
toward the middle of the black hole.  To get back out, you need rocket engines
strong enough to prevent tomorrow from happening.

Complicating matters, physicists debate whether general relativity is correct about the event horizon.  A future [Theory of
Everything](https://en.wikipedia.org/wiki/Theory_of_everything) could describe
very different physics there. See
[Fuzzballs](https://www.youtube.com/watch?v=351JCOvKcYw) and
[Firewalls](https://www.youtube.com/watch?v=4uQF9Egc-fM&t=14s) for example.
Black Hole Lab, being a general relativity simulation, inherits the **smooth no
drama** view.

### The Black Hole Inner (Cauchy) Horizon

The next magenta colored ring inside the event horizon is the Cauchy or inner
horizon.  This horizon exists in spinning black holes, which is probably every
real black hole in our universe.  Physicists use the shorthand name `r_` for the
inner horizon. If the outer horizon is strange, the inner horizon is bizarre.  A
few of the more mind-boggling aspects to think about:

* The inner horizon is the boundary where the arrow of time stops pointing
  strictly inward and tilts back to point into the future like normal space.  An
  observer can once again move around in 3 dimensions.

* Like the outer horizon, the inner horizon can be both a moment in time or a
  place in space depending on which side you find yourself.

* Unlike the outer horizon, the inner horizon profoundly compresses incoming
  time.  As you get close, the outside universe appears to run ever faster
  relative to your clock.  In general relativity, this effect grows
  exponentially and without bound!  Unfortunately for you, this time compression
  effect also gives incoming light an unbounded energy boost.  Physicists refer
  to this phenomenon as "infinite blueshift". You fry in a bath of future light
  boosted to extreme energy.

* Whether you pass through the inner horizon or just asymptotically approach
  depends on how much angular momentum you have, i.e. how steep is your fall
  inward.

Our understanding of the physics at this layer is even less clear than at the
outer event horizon.  As before, Black Hole Lab let's general relativity be the
only word on the physics.

### The Ring Singularity

If you cross the inner horizon you enter what physicists call `region III` of
the black hole.  This deep in a black hole arguably exists only in the math of
general relativity.  As in ancient maps with uncharted edges, "Here Be Dragons".

Below you in `region III` is the *ring singularity*.  In the math, the ring
singularity contains the mass of the black hole in a skinny spinning donut of
infinite density.  In Black Hole Lab, the ring singularity is mathematically
radius r = 0.  In `region III`, the arrow of time once again points toward the
future and an observer can move around in 3 dimensions.

If you enter `region III` with enough angular momentum, you swing back out
toward the inner horizon, but this time asymptotically approaching from below.









# Other Text

A few things to keep in mind about the event horizon:


## Frame's of Reference

In Einstein's relativity theories, your frame of reference makes all the
difference!  Your *frame of reference* is your local spacetime, e.g. inside your
rocket ship, where your physic experiments measure the speed of light as 'c' in
all directions.

Imagine Bob and Alice falling into the black hole together while we
watch from a safe and much less adventurous location.  We never see them cross
the horizon!  Their light becomes increasingly stretched, aka redshifted, and
we see their clocks run slower and slower.  At some point, Bob and Alice are
just barely above the event horizon but are too dim for us to perceive anymore.
You can think of the light reflecting off them as being "exhausted" from
swimming upstream against the flow of space into the black hole.



### Radio Waves

The orange rings around Alice and Bob show the propagation of their radio
signals, which travel at the speed of light.  Notice how the black hole tugs
strongly on Alice's signal

