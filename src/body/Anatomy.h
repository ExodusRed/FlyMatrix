#pragma once

#include "engine/Math.h"

// Every dimension of the fly, in one place, with its provenance.
//
// These numbers used to be scattered as literals across FlyBody (leg
// geometry), FlyPhysics (masses and collision extents) and body_main (what
// gets drawn). Three copies of the same body meant they could disagree, and
// they did: the renderer drew the trunk from a transform the physics never
// wrote to, so the body and the legs only agreed while the fly stood still.
//
// Units are millimetres, micrograms and seconds throughout, matching the
// physics.
//
// HONESTY ABOUT SOURCES. Each block says where its numbers come from.
// "measured" means a published measurement of real flies. "proportional"
// means a measured total divided up by published ratios. "estimated" means
// nobody measured it for us and the value is chosen to look right and behave
// sensibly -- those are the ones to distrust.
//
//   Adult body length is 2.5-3 mm and width about 2 mm, females larger than
//   males (Animal Diversity Web; SANBI). Body mass is about 1 mg.
//
//   The leg is coxa, trochanter, femur, tibia, tarsus and pretarsus, and the
//   tarsus is subdivided into five tarsomeres ta1-ta5 (Kojima, Cytologia
//   2019; the leg-development literature generally). The femurs of all three
//   legs are similar in length while the tibia is shorter on the forelegs
//   (Appendometer, bioRxiv 2025).
//
//   A complete biomechanical fly carries, besides the legs: two wings, two
//   halteres, a head with eyes and antennae (pedicel, funiculus, arista), a
//   proboscis of rostrum and haustellum, and a segmented abdomen
//   (NeuroMechFly, Wang-Chen et al. 2024). Halteres attach to the metathorax
//   and are the serial homologue of the hind wings; the wing is flat and
//   bilayered, the haltere smaller and globular (Drosophila wing/haltere
//   development literature).
namespace fly::anat {

// --- trunk ------------------------------------------------------------
//
// Proportional. The 2.5 mm total is measured; the split between head,
// thorax and abdomen follows the usual description of a fly in lateral view,
// where the abdomen is the longest of the three and the head the shortest.
//
// The previous model spanned 2.04 mm nose to abdomen tip, about 20% short of
// the real animal, and its abdomen was too round.
constexpr float kBodyLength = 2.50f;

// Half-extents, so these are the numbers both setBoxInertia and the drawn
// ellipsoid take.
constexpr V3 kThoraxHalf{0.50f, 0.33f, 0.34f};
constexpr V3 kAbdomenHalf{0.62f, 0.30f, 0.31f};
constexpr V3 kHeadHalf{0.26f, 0.28f, 0.26f};

// Offsets from the thorax centre, along +X toward the head.
constexpr float kAbdomenX = -0.86f;
constexpr float kAbdomenZ = -0.05f;
constexpr float kHeadX = 0.62f;
constexpr float kHeadZ = 0.04f;

// Masses, micrograms. Measured total (~1 mg) split by volume. The old model
// totalled 1344 ug including legs, a third heavier than a real fly.
constexpr float kThoraxMass = 330.0f;
constexpr float kAbdomenMass = 330.0f;
constexpr float kHeadMass = 80.0f;

// --- head ---------------------------------------------------------------
//
// The compound eyes are large and dominate the head, roughly 700-750
// ommatidia each (anatomical atlas, Genetics 2024). Positions estimated from
// lateral and frontal views.
constexpr V3 kEyeHalf{0.17f, 0.13f, 0.20f};
constexpr V3 kEyeOffset{0.10f, 0.19f, 0.03f};  // y is mirrored per side

// The proboscis: the feeding apparatus, and the reason a fly can eat at all.
// Folded under the head at rest and extended to feed. Modelled as the two
// parts the literature names, rostrum (proximal) and haustellum (distal),
// plus the labellum -- the fleshy paired lobe at the tip that actually
// contacts food.
//
// Estimated. It is drawn in the retracted posture, angled down and slightly
// back under the head.
constexpr V3 kRostrumHalf{0.09f, 0.09f, 0.11f};
constexpr V3 kRostrumOffset{0.10f, 0.0f, -0.24f};
constexpr V3 kHaustellumHalf{0.06f, 0.06f, 0.10f};
constexpr V3 kHaustellumOffset{0.13f, 0.0f, -0.42f};
constexpr V3 kLabellumHalf{0.09f, 0.07f, 0.06f};
constexpr V3 kLabellumOffset{0.15f, 0.05f, -0.53f};  // y mirrored per lobe

// Antennae. Three segments in the fly, of which the funiculus (third
// segment) and its feathery arista are the conspicuous ones. Estimated.
constexpr V3 kFuniculusHalf{0.07f, 0.06f, 0.08f};
constexpr V3 kFuniculusOffset{0.20f, 0.09f, -0.06f};
constexpr float kAristaLength = 0.30f;
constexpr float kAristaRadius = 0.012f;

// --- wings --------------------------------------------------------------
//
// Wing length is close to body length, about 2.2-2.5 mm, and the wing is a
// flat bilayered blade roughly 1.1 mm at its widest. At rest the wings fold
// back over the abdomen and overhang its tip, which is why a fly in side
// view looks longer than its body.
//
// Measured length, estimated posture.
constexpr float kWingLength = 2.30f;
constexpr float kWingWidth = 0.78f;
constexpr float kWingThickness = 0.012f;
// Hinge on the mesothorax, above and behind the middle leg.
constexpr V3 kWingRoot{-0.12f, 0.22f, 0.30f};  // y mirrored per side
// Resting sweep: back along the body and splayed out a little.
constexpr float kWingSweepRad = 0.22f;   // rotation about Z, away from midline
constexpr float kWingTiltRad = 0.12f;    // nose-up tilt of the blade
// Wings are membranous and very light: a fraction of a percent of body mass.
constexpr float kWingMass = 3.0f;

// --- halteres -----------------------------------------------------------
//
// The fly's gyroscopes: club-shaped organs on the metathorax, the serial
// homologue of the hind wings, beating antiphase to the wings and reporting
// body rotation. Small and globular against the flat blade of the wing.
//
// Estimated dimensions, measured role.
constexpr float kHaltereStalk = 0.18f;
constexpr float kHaltereStalkRadius = 0.018f;
constexpr float kHaltereKnobRadius = 0.055f;
// On the posterior thorax just behind and below the wing base. It used
// to sit at x = -0.42, which after the abdomen was lengthened was
// *inside* the abdomen, so the stalks appeared to float free of the fly.
constexpr V3 kHaltereRoot{-0.34f, 0.19f, 0.02f};  // y mirrored per side
constexpr float kHaltereMass = 1.0f;

// --- legs ---------------------------------------------------------------
//
// Proportional, and close to where the hand-set values already were: the leg
// segment ratios in this model were about right and did not need moving.
//
// The tarsus is the part that was wrong. It is five tarsomeres, ta1 the
// longest and the rest tapering, ending in a pretarsus with claws -- not the
// single rigid rod this model used. A real fly's foot is a compliant chain
// that drapes over what it stands on.
constexpr float kCoxaLen = 0.26f;
constexpr float kTrochLen = 0.09f;
constexpr float kFemurLen = 0.54f;
constexpr float kTibiaLen = 0.50f;
constexpr float kTarsusLen = 0.55f;

// Joint limits, radians, in the order ThC, CTr, TrF, FTi, TiTa.
//
// ONE COPY. These used to live as literals in FlyBody.cpp and again as a
// LIMITS array in tools/solve_rest_pose.py, and they drifted apart on four
// joints out of five -- ThC +/-0.9 against +/-1.6, CTr +/-1.6 against
// -2.8/+2.0, FTi +/-2.6 against -1.0/+3.3, TiTa +/-3.0 against -3.3/+1.5.
//
// The rest-pose solver was therefore producing poses that the physics
// immediately rejected: every leg's FTi rest angle came out below the C++
// minimum, the limit constraint shoved all six on the first step, and the fly
// stood at 0.93 mm with four feet down instead of 0.55 with six. The solver
// reads these values out of this header now, so they cannot drift again.
//
// Asymmetric because a real leg's are: a knee has far more flexion than
// hyperextension, and the rest pose has to sit inside them with room to move
// both ways.
constexpr float kJointLimit[5][2] = {
    {-1.6f, 1.6f},   // ThC   protraction / retraction
    {-2.8f, 2.0f},   // CTr   levation / depression
    {-0.9f, 0.9f},   // TrF   femur rotation
    {-1.0f, 3.3f},   // FTi   the knee
    {-3.0f, 3.0f},   // TiTa  the ankle, needs range to lay the tarsus flat
};

// How the tarsus divides. ta1 takes the largest share and ta5 is slightly
// longer than ta4 because it carries the pretarsus.
constexpr int kTarsomereCount = 5;
constexpr float kTarsomereFrac[kTarsomereCount] = {0.40f, 0.17f, 0.14f,
                                                   0.12f, 0.17f};

}  // namespace fly::anat
