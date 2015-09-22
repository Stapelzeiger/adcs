function plot_KF_results(time, x, x_pred, x_est, state_name)

% Compute number of states to plot
num_dim = size(x, 1);

for dim = 1:num_dim
	figure(dim);

	plot(time, x(dim, :));
	hold on;
	plot(time, x_pred(dim, :))
	plot(time, x_est(dim, :));

	xlabel('Time (s)');
	ylabel(sprintf('State variable %s', state_name(dim, :)));
	legend('Exact value', 'Predicted valued', 'Estimated value')
	hold off;
end

end
