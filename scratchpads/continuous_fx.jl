using SymPy;
using LinearAlgebra;
# %%
@syms I,R,S,W, dt
# %%
V = diagm([I,I,I,I,I,I])
# %%
# Position Error update
V[1,2] = I*dt
# Velocity Error update
V[2,3] = -R*S*dt
V[2,4] = -R*dt
V[2,6] = I*dt
# Heading Error update
V[3,3] = I - W*dt
V[3,5] = -I*dt
# %%
V
# I think based on this design we only ever need to update 1,2 2,3 2,4 2,6
