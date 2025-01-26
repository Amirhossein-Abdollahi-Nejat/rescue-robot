% Load all datasets
adult_data = readtable('adult_data.csv');
child_data = readtable('child_data.csv');
cow_data = readtable('cow_data.csv');
ship_data = readtable('ship_data.csv');
donkey_data = readtable('donkey_data.csv');
injured_data = readtable('injured_person_data.csv');

% Extract heart rate and breathing rate
adult_hr = adult_data.HeartRate;
child_hr = child_data.HeartRate;
cow_hr = cow_data.HeartRate;
ship_hr = ship_data.HeartRate;
donkey_hr = donkey_data.HeartRate;
injured_hr = injured_data.HeartRate;

adult_br = adult_data.BreathRate;
child_br = child_data.BreathRate;
cow_br = cow_data.BreathRate;
ship_br = ship_data.BreathRate;
donkey_br = donkey_data.BreathRate;
injured_br = injured_data.BreathRate;

% Combine the data into matrices for comparison
hr_data = [adult_hr, child_hr, cow_hr, ship_hr, donkey_hr, injured_hr];
br_data = [adult_br, child_br, cow_br, ship_br, donkey_br, injured_br];

% Create a figure with two boxplots for HR and BR comparison
figure;

% Boxplot for Heart Rate comparison
subplot(2, 2, 1);  % 2 rows, 2 columns, 1st plot
boxplot(hr_data, 'Labels', {'Adult', 'Child', 'Cow', 'Ship', 'Donkey', 'Injured'}, 'Colors', [0.1 0.2 0.7; 0.7 0.1 0.1; 0.1 0.7 0.1; 0.7 0.7 0.1; 0.7 0.1 0.7; 0.5 0.5 0.5]);
title('Heart Rate Comparison');
ylabel('Heart Rate (bpm)');
grid on;

% Boxplot for Breathing Rate comparison
subplot(2, 2, 2);  % 2 rows, 2 columns, 2nd plot
boxplot(br_data, 'Labels', {'Adult', 'Child', 'Cow', 'Ship', 'Donkey', 'Injured'}, 'Colors', [0.1 0.2 0.7; 0.7 0.1 0.1; 0.1 0.7 0.1; 0.7 0.7 0.1; 0.7 0.1 0.7; 0.5 0.5 0.5]);
title('Breathing Rate Comparison');
ylabel('Breathing Rate (bpm)');
grid on;

% Overlayed Line Plot for Heart Rate comparison
subplot(2, 2, 3);  % 2 rows, 2 columns, 3rd plot
hold on;
plot(adult_hr, 'LineWidth', 2, 'Color', [0.1 0.2 0.7]);  % Adult HR
plot(child_hr, 'LineWidth', 2, 'Color', [0.7 0.1 0.1]);  % Child HR
plot(cow_hr, 'LineWidth', 2, 'Color', [0.1 0.7 0.1]);  % Cow HR
plot(ship_hr, 'LineWidth', 2, 'Color', [0.7 0.7 0.1]);  % Ship HR
plot(donkey_hr, 'LineWidth', 2, 'Color', [0.7 0.1 0.7]);  % Donkey HR
plot(injured_hr, 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);  % Injured HR
title('Heart Rate Comparison (Overlayed)');
xlabel('Sample Index');
ylabel('Heart Rate (bpm)');
legend({'Adult HR', 'Child HR', 'Cow HR', 'Ship HR', 'Donkey HR', 'Injured HR'}, 'Location', 'Best');
grid on;
hold off;

% Overlayed Line Plot for Breathing Rate comparison
subplot(2, 2, 4);  % 2 rows, 2 columns, 4th plot
hold on;
plot(adult_br, 'LineWidth', 2, 'Color', [0.1 0.2 0.7]);  % Adult BR
plot(child_br, 'LineWidth', 2, 'Color', [0.7 0.1 0.1]);  % Child BR
plot(cow_br, 'LineWidth', 2, 'Color', [0.1 0.7 0.1]);  % Cow BR
plot(ship_br, 'LineWidth', 2, 'Color', [0.7 0.7 0.1]);  % Ship BR
plot(donkey_br, 'LineWidth', 2, 'Color', [0.7 0.1 0.7]);  % Donkey BR
plot(injured_br, 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);  % Injured BR
title('Breathing Rate Comparison (Overlayed)');
xlabel('Sample Index');
ylabel('Breathing Rate (bpm)');
legend({'Adult BR', 'Child BR', 'Cow BR', 'Ship BR', 'Donkey BR', 'Injured BR'}, 'Location', 'Best');
grid on;
hold off;

% Adjust layout for better presentation
sgtitle('Comparison of Heart Rate and Breathing Rate', 'FontSize', 16, 'FontWeight', 'bold');

% Display Range for HR and BR
fprintf('Heart Rate Range:\n');
fprintf('Adult: [%.2f, %.2f]\n', min(adult_hr), max(adult_hr));
fprintf('Child: [%.2f, %.2f]\n', min(child_hr), max(child_hr));
fprintf('Cow: [%.2f, %.2f]\n', min(cow_hr), max(cow_hr));
fprintf('Ship: [%.2f, %.2f]\n', min(ship_hr), max(ship_hr));
fprintf('Donkey: [%.2f, %.2f]\n', min(donkey_hr), max(donkey_hr));
fprintf('Injured: [%.2f, %.2f]\n', min(injured_hr), max(injured_hr));

fprintf('Breathing Rate Range:\n');
fprintf('Adult: [%.2f, %.2f]\n', min(adult_br), max(adult_br));
fprintf('Child: [%.2f, %.2f]\n', min(child_br), max(child_br));
fprintf('Cow: [%.2f, %.2f]\n', min(cow_br), max(cow_br));
fprintf('Ship: [%.2f, %.2f]\n', min(ship_br), max(ship_br));
fprintf('Donkey: [%.2f, %.2f]\n', min(donkey_br), max(donkey_br));
fprintf('Injured: [%.2f, %.2f]\n', min(injured_br), max(injured_br));
