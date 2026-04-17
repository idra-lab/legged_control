# ====================================
#         TD-Learning for Safety-V(x)
# ====================================

import os
import time
import copy
import math
import pickle
import numpy as np
from tqdm import tqdm

import jax
import jax.numpy as jnp
import flax.linen as nn
from flax.training import train_state
import optax

# ====================================
#      Environment Configuration
# ====================================
os.environ["XLA_FLAGS"] = os.environ.get("XLA_FLAGS", "") + " --xla_gpu_triton_gemm_any=True"
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "True"

# ====================================
#            Load Dataset
# ====================================
full_path = os.path.realpath(__file__)
folder_path = os.path.dirname(full_path)
data_path = folder_path + '/observation_datasets/observations_rl_controller_250_high_pushes_3_5_original_radius.npy'
model_path = folder_path + '/models/VF_rl_controller_250_high_pushes_3_5_original_radiusH.pkl'

data = np.load(data_path)

print("Dataset loaded:", data.shape)

input_dim = 30

states = data[:, :, :input_dim]
next_states = data[:, :, input_dim:2 * input_dim]
dones = data[:, :, -2]
capt_p = data[:, :, -1]

def fill_padding_with_last_valid(states, next_states, dones, capt_p):
    states_filled = np.copy(states)
    next_states_filled = np.copy(next_states)
    dones_filled = np.copy(dones)
    capt_p_filled = np.copy(capt_p)

    num_episodes, T, _ = states.shape

    for ep in range(num_episodes):
        # Trova primo indice dove compare uno stato di solo zeri
        zero_mask = np.all(states[ep] == 0, axis=1)
        if not np.any(zero_mask):
            continue  # nessun padding, salta episodio

        first_zero_idx = np.argmax(zero_mask)

        # Prendi ultimo stato valido
        last_valid_state = states[ep, first_zero_idx - 1]
        
        for t in range(first_zero_idx, T):
            states_filled[ep, t] = last_valid_state
            next_states_filled[ep, t] = last_valid_state
            dones_filled[ep, t] = 0
            capt_p_filled[ep, t] = 1.0

    return states_filled, next_states_filled, dones_filled, capt_p_filled

# Applica
states, next_states, dones, capt_p = fill_padding_with_last_valid(states, next_states, dones, capt_p)

""" print(capt_p[98,:])
print(dones[98,:])
exit() """

""" # Esempio: trova gli indici degli episodi con almeno un 1
episode_with_done = np.where(np.any(dones == 1, axis=1))[0]
print("Episodi con almeno un 'done':", episode_with_done)
exit() """

print("Observation dataset shape:", states.shape)

# ====================================
#         State Normalization
# ====================================
def normalize_states(states):
    mean = np.mean(states, axis=(0, 1))
    std = np.std(states, axis=(0, 1))
    return (states - mean) / (std + 1e-8), mean, std

states, mean, std = normalize_states(states)
next_states, _, _ = normalize_states(next_states)

# ====================================
#         Convert to JAX tensors
# ====================================
states = jnp.array(states, dtype=jnp.float32)
next_states = jnp.array(next_states, dtype=jnp.float32)
dones = jnp.array(dones, dtype=jnp.float32)
capt_p = jnp.array(capt_p, dtype=jnp.float32)

# ====================================
#       Define Neural Network
# ====================================
class ValueNetwork(nn.Module):
    @nn.compact
    def __call__(self, x):
        x = nn.Dense(512)(x)
        x = nn.LayerNorm()(x)
        x = nn.elu(x)
        x = nn.Dense(256)(x)
        x = nn.LayerNorm()(x)
        x = nn.elu(x)
        x = nn.Dense(128)(x)
        x = nn.LayerNorm()(x)
        x = nn.elu(x)
        # Output initialized to 1 (probability of survival)
        x = nn.Dense(1, kernel_init=nn.initializers.zeros, bias_init=nn.initializers.ones)(x)
        #x = nn_flax.sigmoid(x)
        # x = nn.Dense(1)(x)
        return x.squeeze(-1)
    

# ====================================
#    Initialize / Load Model Params
# ====================================
key = jax.random.PRNGKey(42)
model = ValueNetwork()
params = model.init(key, jnp.ones((1, input_dim)))


load_parameters = False

if load_parameters:
    try:
        with open(model_path, 'rb') as f:
            loaded = pickle.load(f)
            params, mean, std = loaded['model_params'], loaded['mean'], loaded['std']
        print("Loaded model parameters.")
    except FileNotFoundError:
        print("Model file not found. Starting from scratch.")

target_params = copy.deepcopy(params)

# ====================================
#       Optimizer & Train State
# ====================================
learning_rate = 3e-4
tau = 0.01
optimizer = optax.adam(learning_rate)

class TrainState(train_state.TrainState):
    pass

state = TrainState.create(apply_fn=model.apply, params=params, tx=optimizer)

# ====================================
#           Loss Function
# ====================================
""" def loss_fn(params, batch_states, batch_next_states, batch_dones, batch_cp, target_params):
    # compute mask: 1 se stato valido, 0 se padding
    valid_mask = jnp.any(batch_states != 0, axis=1).astype(jnp.float32)

    # usual target
    V_s = model.apply(params, batch_states)
    V_next = jax.lax.stop_gradient(model.apply(target_params, batch_next_states))
    indicators = 1.0 - batch_dones
    target = indicators * ((1 - batch_cp) * V_next + batch_cp)

    loss = jnp.square(V_s - target) * valid_mask
    return jnp.sum(loss) / jnp.sum(valid_mask) """

def loss_fn(params, batch_states, batch_next_states, batch_dones, batch_cp, target_params):
    V_s = model.apply(params, batch_states)
    V_next = jax.lax.stop_gradient(model.apply(target_params, batch_next_states))
    #V_next = jax.numpy.clip(V_next,0,1)
    indicators = 1.0 - batch_dones
    target = indicators * ((1-batch_cp)*V_next + batch_cp)
    # jax.debug.print("target: {x}", x=target)
    # target = indicators * V_next
    return jnp.mean(jnp.square(V_s - target))

@jax.jit
def train_step(state, batch_states, batch_next_states, batch_dones, batch_cp, target_params):
    loss, grads = jax.value_and_grad(loss_fn)(state.params, batch_states, batch_next_states, batch_dones, batch_cp, target_params)
    state = state.apply_gradients(grads=grads)
    return state, loss

@jax.jit
def update_target_params(target_params, params, tau):
    return jax.tree_util.tree_map(lambda t, p: tau * p + (1 - tau) * t, target_params, params)

# ====================================
#          Batch Sampling
# ====================================
def closest_power_of_2(n):
    return 2 ** (n.bit_length() - 1)

def get_batches(states, next_states, dones, capt_p, batch_size, rng):
    batch_size = closest_power_of_2(batch_size)
    num_episodes, num_steps, _ = states.shape
    episodes_per_batch = math.ceil(batch_size / num_steps)
    num_batches = num_episodes // episodes_per_batch

    rng, subkey = jax.random.split(rng)
    indices = jax.random.permutation(subkey, num_episodes)[:num_batches * episodes_per_batch]
    indices = indices.reshape(num_batches, episodes_per_batch)

    def extract(idx):
        s = states[idx].reshape(-1, states.shape[-1])[:batch_size]
        ns = next_states[idx].reshape(-1, next_states.shape[-1])[:batch_size]
        d = dones[idx].reshape(-1)[:batch_size]
        cp = capt_p[idx].reshape(-1)[:batch_size]
        return s, ns, d, cp

    batches = jax.vmap(extract)(indices)
    return batches, rng

# ====================================
#           Training Loop
# ====================================
epochs = 800
batch_size = 512
rng = jax.random.PRNGKey(int(time.time()))
losses, min_losses, max_losses = [], [], []

with tqdm(range(epochs), desc="Epoch") as pbar:
    for epoch in pbar:
        start_time = time.time()
        rng, subkey = jax.random.split(rng)
        batches, rng = get_batches(states, next_states, dones, capt_p, batch_size, subkey)

        epoch_loss = 0
        batch_losses = []

        for batch_states, batch_next_states, batch_dones, batch_cp in zip(*batches):
            state, loss = train_step(state, batch_states, batch_next_states, batch_dones, batch_cp, target_params)
            target_params = update_target_params(target_params, state.params, tau)
            epoch_loss += loss
            batch_losses.append(loss)

        epoch_avg = epoch_loss / batches[0].shape[0]
        losses.append(epoch_avg)
        min_losses.append(np.min(batch_losses))
        max_losses.append(np.max(batch_losses))
        pbar.set_postfix({"Loss": f"{epoch_avg:.2e}", "Hz": f"{1 / (time.time() - start_time):.2f}"})

# ====================================
#           Save Model
# ====================================
with open(model_path, 'wb') as f:
    pickle.dump({'model_params': state.params, 'mean': mean, 'std': std}, f)
print("Model saved at", model_path)

# ====================================
#       Value Estimate Example
# ====================================
sample_state = states[5, 0, :]
v_est = model.apply(state.params, sample_state)
print(f"Estimated V(x) (prob. survival) for good state: {v_est.item():.4f}")

sample_state = states[239, 0, :]
v_est = model.apply(state.params, sample_state)
print(f"Estimated V(x) (prob. survival) for bad state: {v_est.item():.4f}")


# ====================================
#       Value Estimate Statistics
# ====================================

# Stimiamo V(x) per ogni primo stato di ogni episodio
all_v_estimates = jnp.array([model.apply(state.params, s[0]) for s in states])

# Identifica episodi che hanno avuto una terminazione
terminated_mask = jnp.any(dones == 1, axis=1)
non_terminated_mask = ~terminated_mask

# Seleziona stime di V(x) per i due gruppi
v_est_terminated = all_v_estimates[terminated_mask]
v_est_non_terminated = all_v_estimates[non_terminated_mask]

# Calcola statistiche
mean_terminated = jnp.mean(v_est_terminated)
std_terminated = jnp.std(v_est_terminated)

mean_non_terminated = jnp.mean(v_est_non_terminated)
std_non_terminated = jnp.std(v_est_non_terminated)

# Stampa risultati
print("\n===== Value Estimate Statistics =====")
print(f"Episodes with termination   : n = {v_est_terminated.shape[0]}")
print(f"Mean V(x): {mean_terminated:.4f}, Std: {std_terminated:.4f}")
print()
print(f"Episodes without termination: n = {v_est_non_terminated.shape[0]}")
print(f"Mean V(x): {mean_non_terminated:.4f}, Std: {std_non_terminated:.4f}")

print("\n===== All V Estimates for Terminated Episodes =====")
print(v_est_terminated)
print("=======================================")