# todo

implementing n way unions and intersections.
This is the n way union that the VTK adapter uses:
https://github.com/bluelightning32/walnut/blob/2e8ef4945c4dd3d79fdad1845417dc867c2f00b4/include/walnut/boolean_operation_filter.h#L21


This is the 2 way union that walnut-javascript is using:
https://github.com/bluelightning32/walnut/blob/master/include/walnut/bsp_visitor.h#L123

I started with a 2 way union for simplicity

how to design the walnut-javascript interface for an n way union.

It's also possible to make more complex filters. You could make a filter for (A | B) & C, where A, B, and C are polyhedrons and | means union and & means intersection.
In that case, A, B, and C can be added to the same BSP, then the filter tells Walnut which branches of the BSP to look at and which facets to output.


Unfortunately subtraction can't be easily combined, because its operand has to be inverted.
