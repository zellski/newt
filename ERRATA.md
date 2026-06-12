# Thesis Errata

Corrections to the thesis report ("An Implementation of the Spacetime
Constraints Approach to the Synthesis of Realistic Motion", Pär Winzell,
November 2004), found during a 2026 review of the paper and this code.

## §3.1.2, eq. (3.2): sign of the gravity term in the Lagrangian

The falling-ball example states the Lagrangian as

    L = T − V = m (ẏ²/2 − G y)

and then derives `ÿ = G`. Applying Lagrange's equation (3.1) to that L
actually yields `ÿ = −G`. With G the *signed* acceleration of gravity
(the code passes −9.81), the potential of a constant force `mG ŷ` is
`V = −mGy`, so the example should read

    L = T − V = m (ẏ²/2 + G y)

after which `ÿ = G` follows as claimed. The slip is confined to this
introductory example: the gravity terms actually implemented (eq. 4.31
and the Cartesian `∂L/∂y = G·MΣ` in `Creature::BuildSweep`) are
consistent with the correct sign, as is eq. (3.2)'s Newtonian form.

## §4.4.2: interior bound-sampling points

The text prescribes sampling interior bound constraints at u = {1/3, 2/3}.
The released code sampled at {0.3, 0.7} in the Hermite basis and at an
asymmetric {0.2, 0.7} in the Hermlet basis; as of the 2026 bug-fix sweep
the code matches the thesis.
