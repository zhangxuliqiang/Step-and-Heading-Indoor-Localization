function [final_P, final_q] = MeasurementUpdate(error, prior_est, prior_P, cov_mat, H)

    S = H*prior_P*H' + cov_mat;
    K = (prior_P*H') / S;
    post_P = prior_P - K*S*K';
    post_est = prior_est + K* error;

    final_q  = post_est/norm(post_est);
%     final_q = final_q * sign(final_q(1));
    J = (1/norm(post_est)^3)*(post_est*post_est');
    final_P = J*post_P*J';
end