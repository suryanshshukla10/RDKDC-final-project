function q = pick_thetas(thetas, q_current)
diff = abs(sum(kron(q_current, ones(1,8)) - thetas));
[m, i] = min(diff);
 
q = thetas(:, i);
end