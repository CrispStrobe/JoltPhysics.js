# Explicit internal-edge pair bindings

Package candidate `1.1.0-bw.10` adds
`PhysicsSystem.SetSimInternalEdgeRemovalPair(sourceBodyID, surfaceBodyID, enabled)`
and `ClearSimInternalEdgeRemovalPairs()`. The native policy defaults to empty;
only selected pairs use enhanced internal-edge removal with the requested
surface as shape 2. Body flags, solver body order and part-pair filters remain
unchanged.

Configure at an externally serialized idle-world point. The boolean API refuses
mutation during Update, callback reentry, contention or incompatible custom
collide-callback mode. Changes invalidate both pair participants' contact caches;
unassignment and destruction purge affected preferences before ID reuse.

The pair map is immutable session configuration for consumers using checkpoints.
It is not serialized by SaveState. Retain the same configuration and body identity
across replay; reconfiguration requires fresh checkpoints. A new world starts empty.

The package retains the previous scalar/SIMD, debug, multithread, external-Wasm
and asm entry points. Every declared file and export must be present in the
packed artifact; qualifying only the two Lite product imports does not qualify
the other advertised artifacts. Declaration wrappers and component notices are
part of that completeness check.

Source and native/Wasm controls cover selected/unselected pairs, both creation
orders, filters, caches, lifecycle, callbacks and replay. Actual direct-VM
qualification is limited to RimRun and its swapped-ballast control. A release
requires admitted canonical native/binding source commits, complete packed
artifacts and their recorded qualification; source preparation alone does not
authorize a consumer pin. No browser, hardware, performance or general-model
claim follows from those bounded controls.
