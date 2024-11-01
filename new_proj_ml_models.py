import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.svm import SVC
from sklearn.ensemble import VotingClassifier
from sklearn.neural_network import MLPClassifier
from imblearn.over_sampling import SMOTE
from sklearn.model_selection import GridSearchCV
from sklearn.ensemble import BaggingClassifier


# Load the dataset from CSV file
df = pd.read_csv('dataset.csv')
df.dropna(inplace=True)

# Preprocessing
# Convert categorical 'status' to numeric
df = df[df['status'] != ' Unknown']
le = LabelEncoder()
df['status'] = le.fit_transform(df['status'])
df_class0 = df[df['status'] == 0]
df_class1 = df[df['status'] == 1]
df_class2 = df[df['status'] == 2]

# print(df_class0.shape[0])
# print(df_class1.shape[0])
# print(df_class2.shape[0])

n = max(df_class0.shape[0],df_class1.shape[0],df_class2.shape[0])
print(n)

class_0_under = df_class0.sample(n=n,replace=True)
class_1_under = df_class1.sample(n=n,replace=True)
class_2_under = df_class2.sample(n=n,replace= True)

df = pd.concat([class_0_under, class_1_under, class_2_under], axis=0)
 

# Define features and target variable
X = df[['timestamp', 'packet_dropped', 'payload_size', 'snr', 'rssi', 'retransmission_count', 'retransmission_delay']]
y = df['status']

# Split the dataset into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
smote = SMOTE(random_state=42)
X_train_smote, y_train_smote = smote.fit_resample(X_train, y_train)

# Initialize individual models with class weights
dt_classifier = DecisionTreeClassifier(class_weight='balanced', random_state=42)
rf_classifier = RandomForestClassifier(class_weight='balanced', random_state=42)

# Create a pipeline for SVM with probability enabled
svm_pipeline = Pipeline([
    ('scaler', StandardScaler()),
    ('svm', SVC(kernel='linear', class_weight='balanced', probability=True, random_state=42))
])

# Initialize ANN with some basic parameters
ann_classifier = MLPClassifier(hidden_layer_sizes=(75, 50), max_iter=500, random_state=42)

# Initialize the Voting Classifier with soft voting, including ANN
ensemble_model = VotingClassifier(
    estimators=[
        ('decision_tree', dt_classifier),
        ('svm', svm_pipeline),
        ('random_forest', rf_classifier),
        ('ann', ann_classifier)
    ],
    voting='hard',
    weights=[3, 1, 4, 8]
)
param_grid = {
    'weights': [
        [1, 1, 1, 3],
        [2, 1, 1, 5],
        [3, 2, 1, 4],
        [2, 2, 1, 5],
        [3,1,4,8]
    ]
}



# Fit the ensemble model
ensemble_model.fit(X_train_smote, y_train_smote)

# Make predictions with the ensemble model
y_pred_ensemble = ensemble_model.predict(X_test)

# Evaluate the individual and ensemble models
print("\nClassification Report DT:")
print(classification_report(y_test, dt_classifier.fit(X_train_smote, y_train_smote).predict(X_test)))

print("\nClassification Report SVM:")
print(classification_report(y_test, svm_pipeline.fit(X_train_smote, y_train_smote).predict(X_test)))

print("\nClassification Report RF:")
print(classification_report(y_test, rf_classifier.fit(X_train_smote, y_train_smote).predict(X_test)))

print("\nClassification Report ANN:")
print(classification_report(y_test, ann_classifier.fit(X_train_smote, y_train_smote).predict(X_test)))

print("\nClassification Report Ensemble:")
print(classification_report(y_test, y_pred_ensemble))

grid_search = GridSearchCV(estimator=ensemble_model, param_grid=param_grid, scoring='f1_macro', cv=3)
grid_search.fit(X_train, y_train)

# Print the best weights found
print("Best weights:", grid_search.best_params_['weights'])

# Evaluate the tuned ensemble
y_pred_tuned_ensemble = grid_search.best_estimator_.predict(X_test)
print("\nClassification Report Tuned Ensemble:")
print(classification_report(y_test, y_pred_tuned_ensemble))


bagging_classifier = BaggingClassifier(n_estimators=100, random_state=42)
y_pred_tuned_bagged_ensemble = grid_search.best_estimator_.predict(X_test)
print("\nClassification Report Tuned Bagged Ensemble:")
print(classification_report(y_test, y_pred_tuned_bagged_ensemble))
