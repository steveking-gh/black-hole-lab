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
relativity.  To the precision of the numerical integration, the effects shown in
Black Hole Lab are really what the math of general relativity says will happen.
In some cases, the calculations stop at a limit, such as 10 billion for the
ratio of the distant clock to Alice and Bob's clock. While the predictions of
general relativity have been proven correct countless times, physicists agree
that general relativity *cannot* be the final word on black holes.  Nobody knows
yet what would *actually* happen to Alice and Bob inside a black hole.

## Starting Up

Black Hole Lab divides the screen into 3 parts.  On the left are most of the
settings.  In the middle is the selectable frame of reference (FoR) view or the
global "foliation" chart.  On the right is the 2 dimensional equatorial view.

![Starting Up](assets/images/startup.png)

## Basic Controls

To start, click **Play** in the top left.  You'll see Alice and Bob start moving
and broadcasting their signals.  By default, Alice orbits the black hole
while Bob takes a rather uninteresting plunge inward.

![Basic Controls](assets/images/basic_controls.png)

Click **Reset** to return the simulation to its starting point.  You can also
play/pause the simulation with the space bar.  The keyboard arrow keys **single
step** the simulation forward and backward in time.

You can also save and load your simulation using the associated buttons.

## The Equatorial Plane View

The right hand side shows a top-down chart of the black hole and surrounding
space.  The black hole looks like a colored bullseye while Alice and Bob are the
amber and mint dots respectively.  The black hole spins counter-clockwise.

![Equatorial View](assets/images/equatorial_view.png)

## The Frame of Reference View

The middle pane of Black Hole Lab is the frame of reference view.  You can
choose whether this pane shows Alice or Bob's frame of reference, or you can
show a 1D+1 or a 2D+1 "foliation" chart.  First, let's discuss what we mean by
"frame of reference".

### What is a Frame of Reference?

Your *frame of reference* (FoR) is your local reality where your experiments
measure the speed of light as `c` in all directions.  Your FoR is the all
important determinant of distance, time and the order of events.  For example,
two stopwatches in different FoRs could measure the same event as taking a
million years or a millisecond.  The universe has no preferred FoR, so neither
stopwatch is more correct than the other.

### Frame of Reference vs Chart

Black Hole Lab shows several *charts* such as the `EQUATORIAL PLANE` view.  The
difference between these charts and a proper FoR is that a chart does *not* show
the speed of light as constant in all directions.  You'll see light traveling
"upstream" against gravity as moving more slowly than light aimed "downstream"
into the black hole.  These charts treat light akin to a sound wave, yet still
respect the mathematical rigor of general relativity.

### Coordinate Time vs Proper Time

*Coordinate time* is the artificial "chart clock" used to step the mathematics
of the simulation.  Fudging just slightly, one second of coordinate time is our common notion of one second.

*Proper time* is the time measured in an observer's FoR, e.g. Bob's wristwatch.

To get a sense of the difference, imagine Bob falling into the black hole while
Alice observes from far away.  Let's compare Alice's proper time, Bob's proper time and the coordinate time.

* In her proper time, Alice sees Bob slow as he nears the event horizon and his
  watch ticking ever more slowly.  Wait as she might, Alice never sees Bob cross
  the horizon as his light becomes too stretched (redshifted) to detect.  To
  her, Bob's watch appears all but stopped.
* In his proper time, Bob dives through the event horizon and into the black
  hole at tremendous uninterrupted speed.  His watch says the whole trip took
  just a few minutes.
* In coordinate time, Bob moves smoothly across the event horizon without
  slowing down.

### Global Foliation Charts

The default view in the FoR pane is the `Global Foliation Chart 1D+1`.

![Global Foliation Chart 1D+1](assets/images/global_foliation_1d1.png)

The global foliation chart shows the simulation's view of events.  Notice the
`1D+1` notation.  The `1D` means this view shows 1 dimension of space
horizontally.  Physicists judiciously choose that dimension to be the radial
separation from ring singularity.  The `+1` is the time dimension shown
vertically.  Bob and Alice's FoR views share this `1D+1` style.

The other unique view is the **Global Foliation Chart 2D+1**

![Global Foliation Chart 2D+1](assets/images/global_foliation_2d1.png)

In this view, we get two dimensions of space like the Equatorial view, with time
pointing vertically up.

### Bob and Alice's FoR

The last two options we can show are **Bob's Frame Of Reference 1D+1** and
likewise for Alice.  These views show `1D+1` reality as measured by the
observer.  The math gives us an oracle-like view since we can see places and
events in the observer's *future*.  Of course a real Alice and Bob would only
perceive their *now*.

![Bob 1D+1](assets/images/bob_for_1d1.png)

### Light Cones

All the graphs above feature cone shapes centered on the observers.  These are
**light cones**.  Light cones provide a helpful way to reason about observers
and are common in diagrams on relativity.  The cone *below* the observer shows
the region of spacetime where light from *any past event* can reach the
observer.  The cone *above* the observer shows the region of spacetime where
light from the observer can reach *any future event*.  As a universal
convention, physicists use a 45 degree line to represent the speed of light:
travels one unit of space horizontally per one unit of time vertically.

We pick on "light from an event" here, but this really means *any*
cause-and-effect influence whatsoever.  An event outside your light cone
*cannot* affect your present reality.  Naturally as time goes by, an event
outside your light cone *now* might not be outside your light cone *later*.

### Light Cone Tilting

If you look at the global foliation view above, you'll notice that Bob and
Alice's light cones appear tilted toward the black hole.  This is a real
physical effect of the warping of spacetime by gravity.  It's quite fair to say
that gravity *pulls on your future*.

On the other hand, Bob's FoR view shows his light cone rigidly at 45 degrees.
From Bob's frame of reference, light moves at `c` in all directions.  Things
might appear warped *within* Bob's light cone, but light itself moves
unfailingly at `c` in an observer FoR.

Note that Bob's FoR is a *frame of reference* while the global foliation view
that shows tilted light cones is a *chart* to help us understand.

## Looking At Physics with Black Hole Lab

The following sections show some interesting features of general relativity.
We'll use Sagittarius A*, which scientists believe is the real black hole at the
center of our galaxy.

### Outside the Black Hole

Uncheck Alice, then let's watch Bob holding still some ways outside the black
hole.  Enable Bob's "Transmit Signal", then set Bob to "Static" and "At rest
here".

![Bob Static 1](assets/images/bob_static_1.png)

Play the simulation for a bit and you'll see Bob transmitting.  The simulation
only tracks the fixed number of Bob's transmission wavefronts as set by the
"Wavefronts kept" slider control.

![Bob Static 2](assets/images/bob_static_2.png)

We can see the black hole capturing a good portion of the radio waves.  If you
look closely, you can see the waves face three different fates depending on the
angle at which they crossed the event horizon.  More about that later.

In the `EQUATORIAL PLANE` view, zoom in to Bob with the mouse wheel.

![Bob Static 3](assets/images/bob_static_3.png)

Here we can see the gravity and frame dragging of Sag A* pulling strongly on
Bob's radio transmissions.  The `EQUATORIAL PLANE` *chart* shows the radio waves
heading downstream with the flow of space moving much faster than those aimed
away from the black hole.  Another observer checking the signal in various
places from their own FoR would measure all wavefronts moving at speed `c`, but
with the "fast" waves blueshifted and the "slow" waves redshifted.  Bob notices
nothing special in his FoR and his transmission looks identical in all
directions.

At this spot we also see from the `a_thrust` stat in Bob's info box that his
rocket is accelerating at 188209 times earth gravity to keep him stationary
against the pull of gravity.  This is very analogous to swimming upstream
against the fast current of space pouring into the black hole. On the other
hand, we notice that the `Tidal` force on Bob, aka the famous
*spaghettification* effect, is only 0.00001 g/m, which would be unnoticeable.

### Receiving Radio Signals

Bob and Alice record when they receive the other's radio signal.  Receiving a signal appears as a special tick mark on the observer's world line.

![Signal Reception](assets/images/receiving_signal.png)

In this `EQUATORIAL PLANE` view, we see 3 triangular tick marks on Alice's world line that mark exactly when she received the first 3 wavefronts that have already passed her in space.

The observer FoR and global foliation views also show signal receptions.  Here's
Alice's FoR.  We see 3 turquoise color wavefronts have passed Alice and are now
in her past.  Several more are in her future.  Because this is an FoR, the waves
move past Alice at the speed of light and appear as 45 degree angle lines by
definition.

![Alice Reception](assets/images/alice_receive_1.png)


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

Next inside the ergosphere is the cyan *event horizon*.  The event horizon
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
  strictly inward and tilts back to point into the future like normal space.

* Like the outer horizon, the inner horizon can be either a moment in time or a
  place in space depending on which side you find yourself.

* The inner horizon has two *branches*, so to speak, which we'll call *near* and
  *far*. If you have high angular momentum, you'll encounter the *far* branch
  where the inner horizon profoundly compresses incoming time.  As you get
  close, the outside universe appears to run ever faster relative to your clock.
  In general relativity, this effect grows exponentially and without bound!
  Unfortunately for you, this time compression effect also gives incoming light
  an unbounded energy boost. Physicists refer to this phenomenon as "infinite
  blueshift". You fry in a bath of future light boosted to extreme energy.

  With low angular momentum, you fall through the *near* branch of the horizon.
  In this case, you don't linger from the outside universe's perspective.  You
  will unfortunately be fried by the "infinite blueshift" of light emitted from
  everything that fell in before you on this path.

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

If you enter `region III` with enough angular momentum, you swing back outward
toward the underside of the inner horizon.  This approach is asymptotic in the
simulation's coordinate time, so the simulation never shows a crossing back
through r_.  However, in the proper time of the observer, this crossing happens
quickly.  According to general relativity, the observer finds himself in an
entirely different spacetime and *not* the black hole!



