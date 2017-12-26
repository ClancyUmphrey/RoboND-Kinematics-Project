# File: matrices2png.py

# Line width: 90

# Author: Clancy Umphrey

# Description:
# Generates images of matrices and equations from kinematics.py for the project write-up.

#-----------------------------------------------------------------------------------------
# Import Modules
#-----------------------------------------------------------------------------------------
from sympy import preview, latex, symbols, simplify
from sympy.matrices import Matrix
from subprocess import check_output

# Load the kinematics module
#-----------------------------------------------------------------------------------------
import sys, os
# Add kinematics directory to the local python path
sys.path.append(os.path.join('..','..','kuka_arm','scripts'))
# 
from kinematics import *

#=========================================================================================
# Setup 
#=========================================================================================
# Map DH symbols to LaTeX
sym2latex = {A0: '\\alpha_0',  a0: 'a_0',  d1: 'd_1',  q1: '\\theta_1',  pi: '\\pi', 
            A1: '\\alpha_1',  a1: 'a_1',  d2: 'd_2',  q2: '\\theta_2', 
            A2: '\\alpha_2',  a2: 'a_2',  d3: 'd_3',  q3: '\\theta_3', 
            A3: '\\alpha_3',  a3: 'a_3',  d4: 'd_4',  q4: '\\theta_4', 
            A4: '\\alpha_4',  a4: 'a_4',  d5: 'd_5',  q5: '\\theta_5', 
            A5: '\\alpha_5',  a5: 'a_5',  d6: 'd_6',  q6: '\\theta_6', 
            A6: '\\alpha_6',  a6: 'a_6',  d7: 'd_7',  q7: '\\theta_7',  }
# Map general total transformation symbols to LaTeX
A, a, d, q = symbols('A a d q')
sym2latex.update({A: '\\alpha_{i-1}', a: 'a_{i-1}', d: 'd_i', q: '\\theta_i'})
# Map end-effector position to LaTeX
sym2latex.update({px:'p_x', py:'p_y', pz:'p_z'})
P = Matrix([px, py, pz])

# Dummy symbol used to hide long matrix elements that are not necessary to display
hide = symbols('hide')
sym2latex.update({hide: '...'})

# Options for dvipng (converts dvi file from LaTeX to png)
dpi = '150'
dviOptions = ['-D',dpi,'-T','tight','-z','9','--truecolor','-bg','transparent']

# Markdown entries to print and paste
mkdwn = '[%s]: images/matrices/%s.png'
mkdwns = []

#=========================================================================================
# Functions
#=========================================================================================
def latex2png(string, name, dvi_options=dviOptions, outTex=False):
    """
    Given a LaTeX formatted string, outputs a PNG image (<name>.png) of the string
    processed with LaTeX.  <name> is also added to mkdwns for printing a markdown file
    reference to the image.  A TEX file (<name>.tex) containing the string will also be
    output if ouTex is set to True.
    """
    outTexFile = name+'.tex' if outTex else None
    preview(string, outputTexFile=outTexFile, filename=name+'.png',
            viewer='file', dvioptions=dvi_options)
    mkdwns.append( mkdwn % ((name,)*2) )

#=========================================================================================
# Matrices and Equations to Output 
#=========================================================================================
#-----------------------------------------------------------------------------------------
# General total transform between adjacent links
#-----------------------------------------------------------------------------------------
Tgen = Transform(A, a, d, q, sym2latex)
latex2png(r'$^{i-1}_{i}T = '+latex(Tgen)+'$', 'Tgen')

#-----------------------------------------------------------------------------------------
# Transformation matrices about each joint
#-----------------------------------------------------------------------------------------
# Generate PNG and corresponding LaTeX
for i in xrange(6):
    I = (i, i+1)
    latex2png(r'$^{%d}_{%d}T = '%I+latex(T['%d_%d'%I].subs(sym2latex))+'$', 'T%d_%d'%I)

#-----------------------------------------------------------------------------------------
# Generalized homogeneous transformation between base_link and gripper_link from ee pose 
#-----------------------------------------------------------------------------------------
# URDF to DH Convention Correction
latex2png(r'$^{URDF}_{DH}R = R_Z(\pi) \cdot R_Y(-\pi/2)$', 'Rdf2dh_EQ')
latex2png(r'$^{URDF}_{DH}R = '+latex(R['df2dh'])+'$', 'Rdf2dh')
# Corrected Rotation
latex2png(r'$R_{rpy} = R_Z(y) \cdot R_Y(p) \cdot R_X(r) \cdot \,^{URDF}_{DH}R$',
           'Rrpy_EQ')
# Corrected Transformation
T['B_G'] = R2T(R['rpy'], P).subs(sym2latex)
latex2png(r'$^{B}_{G}T = '+latex(T['B_G'])+'$', 'TB_G')

#-----------------------------------------------------------------------------------------
# Matrices and equations for deriving thetas 
#-----------------------------------------------------------------------------------------
# Inverse Position Kinematics
overarrow = ('\\overrightarrow',)*3
latex2png(r'$%s{wc} = %s{p} - d_7 \cdot %s{n}$'%overarrow, 'wc_EQ')
# Inverse Orientation Kinematics
latex2png(r'$^{0}_{3}R = \,^{0}_{1}R \cdot \,^{1}_{2}R \cdot \,^{2}_{3}R$', 'R0_3_EQ')
latex2png(r'$^{0}_{6}R = \,^{0}_{1}R \cdot \,^{1}_{2}R \cdot \,^{2}_{3}R ' \
          +'\cdot \,^{3}_{4}R \cdot \,^{4}_{5}R \cdot \,^{5}_{6}R$', 'R0_6_EQ')
latex2png(r'$^{0}_{3}R^T \cdot \,^{0}_{6}R = \,^{0}_{3}R^T \cdot  R_{rpy}$',
           'R0_6_Rrpy_EQ')
latex2png(r'$^{3}_{6}R_{lhs} = \,^{3}_{6}R_{rhs}$', 'R3_6_EQ')
R['3_6lhs'][0,0] = hide
R['3_6lhs'][2,0] = hide
R['3_6lhs'][0,1] = hide
R['3_6lhs'][2,1] = hide
latex2png(r'$^{3}_{6}R_{lhs} = '+latex(simplify(R['3_6lhs']).subs(sym2latex))+'$',
           'R3_6lhs')
latex2png(r'$\tan{\theta} = \frac{\sin{\theta}}{\cos{\theta}}$', 'tan')
latex2png(r'$\theta_4 = atan2(rhs_{3,3}, -rhs_{1,3})$', 'theta4')
latex2png(r'$\theta_5 = atan2(\sqrt{(rhs_{2,1})^2+(rhs_{2,2})^2}, rhs_{2,3})$', 'theta5')
latex2png(r'$\theta_6 = atan2(-rhs_{2,2}, rhs_{2,1})$', 'theta6')

#=========================================================================================
# Print markdown entries
#=========================================================================================
for m in mkdwns: print m

