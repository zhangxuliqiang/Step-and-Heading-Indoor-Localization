function [post_P, post_est] = MeasurementUpdate(error, prior_est, prior_P, cov_mat, H)

    S = H*prior_P*H' + cov_mat;
    K = (prior_P*H') / S;
    post_P = prior_P - K*S*K';
    post_est = prior_est + K* error;
    post_est = post_est/ norm(post_est);
end