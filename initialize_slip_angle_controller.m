load('alpha_delta_controller_IT7.mat')
[cA_alpha,cB_alpha,cC_alpha,cD_alpha] = tf2ss(cell2mat(shapeit_data.C_tf.Numerator),cell2mat(shapeit_data.C_tf.Denominator));
input.A_alpha = cA_alpha; input.B_alpha = cB_alpha; input.C_alpha = cC_alpha; input.D_alpha = cD_alpha;