load('alpha_delta_controller_IT7.mat')
[cA_alpha,cB_alpha,cC_alpha,cD_alpha] = tf2ss(cell2mat(shapeit_data.C_tf.Numerator),cell2mat(shapeit_data.C_tf.Denominator));
input.A_alpha = cA_alpha; input.B_alpha = cB_alpha; input.C_alpha = cC_alpha; input.D_alpha = cD_alpha;


load('low_pass_filter_IT1.mat')
[cA_lpf,cB_lpf,cC_lpf,cD_lpf] = tf2ss(cell2mat(shapeit_data.C_tf.Numerator),cell2mat(shapeit_data.C_tf.Denominator));
input.A_lpf = cA_lpf; input.B_lpf = cB_lpf; input.C_lpf = cC_lpf; input.D_lpf = cD_lpf;