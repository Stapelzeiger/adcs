function plot_KF_results(time, x, x_est, P_est, state_name)

% Compute number of states to plot
num_dim = size(x, 1);

for dim = 1:num_dim
	figure(dim);

	plot(time, x(dim, :))
	hold on;
	errorbar(time, x_est(dim, :), P_est(dim,dim,:))

	xlabel('Time (s)');
	ylabel(sprintf('State variable %s', state_name(dim, :)));
	legend('Exact value', 'Estimated value')
	hold off;
end

end
