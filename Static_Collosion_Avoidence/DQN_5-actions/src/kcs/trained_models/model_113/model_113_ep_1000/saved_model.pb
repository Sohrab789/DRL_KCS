��
��
�
ArgMax

input"T
	dimension"Tidx
output"output_type"!
Ttype:
2	
"
Tidxtype0:
2	"!
output_typetype0	:
2	
^
AssignVariableOp
resource
value"dtype"
dtypetype"
validate_shapebool( �
~
BiasAdd

value"T	
bias"T
output"T" 
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
A
BroadcastArgs
s0"T
s1"T
r0"T"
Ttype0:
2	
Z
BroadcastTo

input"T
shape"Tidx
output"T"	
Ttype"
Tidxtype0:
2	
h
ConcatV2
values"T*N
axis"Tidx
output"T"
Nint(0"	
Ttype"
Tidxtype0:
2	
8
Const
output"dtype"
valuetensor"
dtypetype
.
Identity

input"T
output"T"	
Ttype
q
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:

2	
>
Maximum
x"T
y"T
z"T"
Ttype:
2	
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(�
>
Minimum
x"T
y"T
z"T"
Ttype:
2	

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
�
PartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype�
[
Reshape
tensor"T
shape"Tshape
output"T"	
Ttype"
Tshapetype0:
2	
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0�
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0�
?
Select
	condition

t"T
e"T
output"T"	
Ttype
P
Shape

input"T
output"out_type"	
Ttype"
out_typetype0:
2	
H
ShardedFilename
basename	
shard

num_shards
filename
�
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring ��
@
StaticRegexFullMatch	
input

output
"
patternstring
�
StridedSlice

input"T
begin"Index
end"Index
strides"Index
output"T"	
Ttype"
Indextype:
2	"

begin_maskint "
end_maskint "
ellipsis_maskint "
new_axis_maskint "
shrink_axis_maskint 
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 
-
Tanh
x"T
y"T"
Ttype:

2
�
VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 �"serve*2.9.12v2.9.0-18-gd8ce9f9c3018��
�
QNetwork/dense_14/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*'
shared_nameQNetwork/dense_14/bias
}
*QNetwork/dense_14/bias/Read/ReadVariableOpReadVariableOpQNetwork/dense_14/bias*
_output_shapes
:*
dtype0
�
QNetwork/dense_14/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:	�*)
shared_nameQNetwork/dense_14/kernel
�
,QNetwork/dense_14/kernel/Read/ReadVariableOpReadVariableOpQNetwork/dense_14/kernel*
_output_shapes
:	�*
dtype0
�
&QNetwork/EncodingNetwork/dense_13/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:�*7
shared_name(&QNetwork/EncodingNetwork/dense_13/bias
�
:QNetwork/EncodingNetwork/dense_13/bias/Read/ReadVariableOpReadVariableOp&QNetwork/EncodingNetwork/dense_13/bias*
_output_shapes	
:�*
dtype0
�
(QNetwork/EncodingNetwork/dense_13/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:
��*9
shared_name*(QNetwork/EncodingNetwork/dense_13/kernel
�
<QNetwork/EncodingNetwork/dense_13/kernel/Read/ReadVariableOpReadVariableOp(QNetwork/EncodingNetwork/dense_13/kernel* 
_output_shapes
:
��*
dtype0
�
&QNetwork/EncodingNetwork/dense_12/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:�*7
shared_name(&QNetwork/EncodingNetwork/dense_12/bias
�
:QNetwork/EncodingNetwork/dense_12/bias/Read/ReadVariableOpReadVariableOp&QNetwork/EncodingNetwork/dense_12/bias*
_output_shapes	
:�*
dtype0
�
(QNetwork/EncodingNetwork/dense_12/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:	�*9
shared_name*(QNetwork/EncodingNetwork/dense_12/kernel
�
<QNetwork/EncodingNetwork/dense_12/kernel/Read/ReadVariableOpReadVariableOp(QNetwork/EncodingNetwork/dense_12/kernel*
_output_shapes
:	�*
dtype0
d
VariableVarHandleOp*
_output_shapes
: *
dtype0	*
shape: *
shared_name
Variable
]
Variable/Read/ReadVariableOpReadVariableOpVariable*
_output_shapes
: *
dtype0	

NoOpNoOp
�#
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*�"
value�"B�" B�"
�

train_step
metadata
model_variables
_all_assets

action
distribution
get_initial_state
get_metadata
	get_train_step


signatures*
GA
VARIABLE_VALUEVariable%train_step/.ATTRIBUTES/VARIABLE_VALUE*
* 
.
0
1
2
3
4
5*

_wrapped_policy*

trace_0
trace_1* 

trace_0* 

trace_0* 
* 
* 
K

action
get_initial_state
get_train_step
get_metadata* 
nh
VARIABLE_VALUE(QNetwork/EncodingNetwork/dense_12/kernel,model_variables/0/.ATTRIBUTES/VARIABLE_VALUE*
lf
VARIABLE_VALUE&QNetwork/EncodingNetwork/dense_12/bias,model_variables/1/.ATTRIBUTES/VARIABLE_VALUE*
nh
VARIABLE_VALUE(QNetwork/EncodingNetwork/dense_13/kernel,model_variables/2/.ATTRIBUTES/VARIABLE_VALUE*
lf
VARIABLE_VALUE&QNetwork/EncodingNetwork/dense_13/bias,model_variables/3/.ATTRIBUTES/VARIABLE_VALUE*
^X
VARIABLE_VALUEQNetwork/dense_14/kernel,model_variables/4/.ATTRIBUTES/VARIABLE_VALUE*
\V
VARIABLE_VALUEQNetwork/dense_14/bias,model_variables/5/.ATTRIBUTES/VARIABLE_VALUE*


_q_network*
* 
* 
* 
* 
* 
* 
* 
* 
�
	variables
trainable_variables
regularization_losses
	keras_api
__call__
* &call_and_return_all_conditional_losses
!_encoder
"_q_value_layer*
.
0
1
2
3
4
5*
.
0
1
2
3
4
5*
* 
�
#non_trainable_variables

$layers
%metrics
&layer_regularization_losses
'layer_metrics
	variables
trainable_variables
regularization_losses
__call__
* &call_and_return_all_conditional_losses
& "call_and_return_conditional_losses*
* 
* 
�
(	variables
)trainable_variables
*regularization_losses
+	keras_api
,__call__
*-&call_and_return_all_conditional_losses
._postprocessing_layers*
�
/	variables
0trainable_variables
1regularization_losses
2	keras_api
3__call__
*4&call_and_return_all_conditional_losses

kernel
bias*
* 

!0
"1*
* 
* 
* 
 
0
1
2
3*
 
0
1
2
3*
* 
�
5non_trainable_variables

6layers
7metrics
8layer_regularization_losses
9layer_metrics
(	variables
)trainable_variables
*regularization_losses
,__call__
*-&call_and_return_all_conditional_losses
&-"call_and_return_conditional_losses*
* 
* 

:0
;1
<2*

0
1*

0
1*
* 
�
=non_trainable_variables

>layers
?metrics
@layer_regularization_losses
Alayer_metrics
/	variables
0trainable_variables
1regularization_losses
3__call__
*4&call_and_return_all_conditional_losses
&4"call_and_return_conditional_losses*
* 
* 
* 

:0
;1
<2*
* 
* 
* 
�
B	variables
Ctrainable_variables
Dregularization_losses
E	keras_api
F__call__
*G&call_and_return_all_conditional_losses* 
�
H	variables
Itrainable_variables
Jregularization_losses
K	keras_api
L__call__
*M&call_and_return_all_conditional_losses

kernel
bias*
�
N	variables
Otrainable_variables
Pregularization_losses
Q	keras_api
R__call__
*S&call_and_return_all_conditional_losses

kernel
bias*
* 
* 
* 
* 
* 
* 
* 
* 
�
Tnon_trainable_variables

Ulayers
Vmetrics
Wlayer_regularization_losses
Xlayer_metrics
B	variables
Ctrainable_variables
Dregularization_losses
F__call__
*G&call_and_return_all_conditional_losses
&G"call_and_return_conditional_losses* 
* 
* 

0
1*

0
1*
* 
�
Ynon_trainable_variables

Zlayers
[metrics
\layer_regularization_losses
]layer_metrics
H	variables
Itrainable_variables
Jregularization_losses
L__call__
*M&call_and_return_all_conditional_losses
&M"call_and_return_conditional_losses*
* 
* 

0
1*

0
1*
* 
�
^non_trainable_variables

_layers
`metrics
alayer_regularization_losses
blayer_metrics
N	variables
Otrainable_variables
Pregularization_losses
R__call__
*S&call_and_return_all_conditional_losses
&S"call_and_return_conditional_losses*
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
l
action_0_discountPlaceholder*#
_output_shapes
:���������*
dtype0*
shape:���������
w
action_0_observationPlaceholder*'
_output_shapes
:���������*
dtype0*
shape:���������
j
action_0_rewardPlaceholder*#
_output_shapes
:���������*
dtype0*
shape:���������
m
action_0_step_typePlaceholder*#
_output_shapes
:���������*
dtype0*
shape:���������
�
StatefulPartitionedCallStatefulPartitionedCallaction_0_discountaction_0_observationaction_0_rewardaction_0_step_type(QNetwork/EncodingNetwork/dense_12/kernel&QNetwork/EncodingNetwork/dense_12/bias(QNetwork/EncodingNetwork/dense_13/kernel&QNetwork/EncodingNetwork/dense_13/biasQNetwork/dense_14/kernelQNetwork/dense_14/bias*
Tin
2
*
Tout
2	*
_collective_manager_ids
 *#
_output_shapes
:���������*(
_read_only_resource_inputs

	*0
config_proto 

CPU

GPU2*0J 8� *0
f+R)
'__inference_signature_wrapper_126233526
]
get_initial_state_batch_sizePlaceholder*
_output_shapes
: *
dtype0*
shape: 
�
PartitionedCallPartitionedCallget_initial_state_batch_size*
Tin
2*

Tout
 *
_collective_manager_ids
 * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *0
f+R)
'__inference_signature_wrapper_126233538
�
PartitionedCall_1PartitionedCall*	
Tin
 *

Tout
 *
_collective_manager_ids
 * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *0
f+R)
'__inference_signature_wrapper_126233560
�
StatefulPartitionedCall_1StatefulPartitionedCallVariable*
Tin
2*
Tout
2	*
_collective_manager_ids
 *
_output_shapes
: *#
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *0
f+R)
'__inference_signature_wrapper_126233553
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
�
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenameVariable/Read/ReadVariableOp<QNetwork/EncodingNetwork/dense_12/kernel/Read/ReadVariableOp:QNetwork/EncodingNetwork/dense_12/bias/Read/ReadVariableOp<QNetwork/EncodingNetwork/dense_13/kernel/Read/ReadVariableOp:QNetwork/EncodingNetwork/dense_13/bias/Read/ReadVariableOp,QNetwork/dense_14/kernel/Read/ReadVariableOp*QNetwork/dense_14/bias/Read/ReadVariableOpConst*
Tin
2		*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *+
f&R$
"__inference__traced_save_126233767
�
StatefulPartitionedCall_3StatefulPartitionedCallsaver_filenameVariable(QNetwork/EncodingNetwork/dense_12/kernel&QNetwork/EncodingNetwork/dense_12/bias(QNetwork/EncodingNetwork/dense_13/kernel&QNetwork/EncodingNetwork/dense_13/biasQNetwork/dense_14/kernelQNetwork/dense_14/bias*
Tin

2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *.
f)R'
%__inference__traced_restore_126233798��
�
9
'__inference_get_initial_state_126233718

batch_size*(
_construction_contextkEagerRuntime*
_input_shapes
: :B >

_output_shapes
: 
$
_user_specified_name
batch_size
�!
�
%__inference__traced_restore_126233798
file_prefix#
assignvariableop_variable:	 N
;assignvariableop_1_qnetwork_encodingnetwork_dense_12_kernel:	�H
9assignvariableop_2_qnetwork_encodingnetwork_dense_12_bias:	�O
;assignvariableop_3_qnetwork_encodingnetwork_dense_13_kernel:
��H
9assignvariableop_4_qnetwork_encodingnetwork_dense_13_bias:	�>
+assignvariableop_5_qnetwork_dense_14_kernel:	�7
)assignvariableop_6_qnetwork_dense_14_bias:

identity_8��AssignVariableOp�AssignVariableOp_1�AssignVariableOp_2�AssignVariableOp_3�AssignVariableOp_4�AssignVariableOp_5�AssignVariableOp_6�
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*�
value�B�B%train_step/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/0/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/1/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/2/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/3/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/4/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/5/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH�
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*#
valueBB B B B B B B B �
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*4
_output_shapes"
 ::::::::*
dtypes

2	[
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0	*
_output_shapes
:�
AssignVariableOpAssignVariableOpassignvariableop_variableIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype0	]

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:�
AssignVariableOp_1AssignVariableOp;assignvariableop_1_qnetwork_encodingnetwork_dense_12_kernelIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:�
AssignVariableOp_2AssignVariableOp9assignvariableop_2_qnetwork_encodingnetwork_dense_12_biasIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:�
AssignVariableOp_3AssignVariableOp;assignvariableop_3_qnetwork_encodingnetwork_dense_13_kernelIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:�
AssignVariableOp_4AssignVariableOp9assignvariableop_4_qnetwork_encodingnetwork_dense_13_biasIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:�
AssignVariableOp_5AssignVariableOp+assignvariableop_5_qnetwork_dense_14_kernelIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype0]

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:�
AssignVariableOp_6AssignVariableOp)assignvariableop_6_qnetwork_dense_14_biasIdentity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype01
NoOpNoOp"/device:CPU:0*
_output_shapes
 �

Identity_7Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^NoOp"/device:CPU:0*
T0*
_output_shapes
: U

Identity_8IdentityIdentity_7:output:0^NoOp_1*
T0*
_output_shapes
: �
NoOp_1NoOp^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6*"
_acd_function_control_output(*
_output_shapes
 "!

identity_8Identity_8:output:0*#
_input_shapes
: : : : : : : : 2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12(
AssignVariableOp_2AssignVariableOp_22(
AssignVariableOp_3AssignVariableOp_32(
AssignVariableOp_4AssignVariableOp_42(
AssignVariableOp_5AssignVariableOp_52(
AssignVariableOp_6AssignVariableOp_6:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
�
�
'__inference_signature_wrapper_126233526
discount
observation

reward
	step_type
unknown:	�
	unknown_0:	�
	unknown_1:
��
	unknown_2:	�
	unknown_3:	�
	unknown_4:
identity	��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCall	step_typerewarddiscountobservationunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
2
*
Tout
2	*
_collective_manager_ids
 *#
_output_shapes
:���������*(
_read_only_resource_inputs

	*0
config_proto 

CPU

GPU2*0J 8� *6
f1R/
-__inference_function_with_signature_126233504k
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0	*#
_output_shapes
:���������`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*_
_input_shapesN
L:���������:���������:���������:���������: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
#
_output_shapes
:���������
$
_user_specified_name
0/discount:VR
'
_output_shapes
:���������
'
_user_specified_name0/observation:MI
#
_output_shapes
:���������
"
_user_specified_name
0/reward:PL
#
_output_shapes
:���������
%
_user_specified_name0/step_type
�L
�
+__inference_polymorphic_action_fn_126233680
time_step_step_type
time_step_reward
time_step_discount
time_step_observationS
@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource:	�P
Aqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource:	�T
@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource:
��P
Aqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource:	�C
0qnetwork_dense_14_matmul_readvariableop_resource:	�?
1qnetwork_dense_14_biasadd_readvariableop_resource:
identity	��8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp�8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp�(QNetwork/dense_14/BiasAdd/ReadVariableOp�'QNetwork/dense_14/MatMul/ReadVariableOpy
(QNetwork/EncodingNetwork/flatten_4/ConstConst*
_output_shapes
:*
dtype0*
valueB"����   �
*QNetwork/EncodingNetwork/flatten_4/ReshapeReshapetime_step_observation1QNetwork/EncodingNetwork/flatten_4/Const:output:0*
T0*'
_output_shapes
:����������
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
(QNetwork/EncodingNetwork/dense_12/MatMulMatMul3QNetwork/EncodingNetwork/flatten_4/Reshape:output:0?QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_12/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_12/MatMul:product:0@QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_12/TanhTanh2QNetwork/EncodingNetwork/dense_12/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource* 
_output_shapes
:
��*
dtype0�
(QNetwork/EncodingNetwork/dense_13/MatMulMatMul*QNetwork/EncodingNetwork/dense_12/Tanh:y:0?QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_13/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_13/MatMul:product:0@QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_13/TanhTanh2QNetwork/EncodingNetwork/dense_13/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
'QNetwork/dense_14/MatMul/ReadVariableOpReadVariableOp0qnetwork_dense_14_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
QNetwork/dense_14/MatMulMatMul*QNetwork/EncodingNetwork/dense_13/Tanh:y:0/QNetwork/dense_14/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:����������
(QNetwork/dense_14/BiasAdd/ReadVariableOpReadVariableOp1qnetwork_dense_14_biasadd_readvariableop_resource*
_output_shapes
:*
dtype0�
QNetwork/dense_14/BiasAddBiasAdd"QNetwork/dense_14/MatMul:product:00QNetwork/dense_14/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������l
!Categorical/mode/ArgMax/dimensionConst*
_output_shapes
: *
dtype0*
valueB :
����������
Categorical/mode/ArgMaxArgMax"QNetwork/dense_14/BiasAdd:output:0*Categorical/mode/ArgMax/dimension:output:0*
T0*#
_output_shapes
:���������T
Deterministic/atolConst*
_output_shapes
: *
dtype0	*
value	B	 R T
Deterministic/rtolConst*
_output_shapes
: *
dtype0	*
value	B	 R d
!Deterministic/sample/sample_shapeConst*
_output_shapes
: *
dtype0*
valueB j
Deterministic/sample/ShapeShape Categorical/mode/ArgMax:output:0*
T0	*
_output_shapes
:\
Deterministic/sample/ConstConst*
_output_shapes
: *
dtype0*
value	B : r
(Deterministic/sample/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: t
*Deterministic/sample/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:t
*Deterministic/sample/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:�
"Deterministic/sample/strided_sliceStridedSlice#Deterministic/sample/Shape:output:01Deterministic/sample/strided_slice/stack:output:03Deterministic/sample/strided_slice/stack_1:output:03Deterministic/sample/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_maskh
%Deterministic/sample/BroadcastArgs/s0Const*
_output_shapes
: *
dtype0*
valueB j
'Deterministic/sample/BroadcastArgs/s0_1Const*
_output_shapes
: *
dtype0*
valueB �
"Deterministic/sample/BroadcastArgsBroadcastArgs0Deterministic/sample/BroadcastArgs/s0_1:output:0+Deterministic/sample/strided_slice:output:0*
_output_shapes
:n
$Deterministic/sample/concat/values_0Const*
_output_shapes
:*
dtype0*
valueB:g
$Deterministic/sample/concat/values_2Const*
_output_shapes
: *
dtype0*
valueB b
 Deterministic/sample/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : �
Deterministic/sample/concatConcatV2-Deterministic/sample/concat/values_0:output:0'Deterministic/sample/BroadcastArgs:r0:0-Deterministic/sample/concat/values_2:output:0)Deterministic/sample/concat/axis:output:0*
N*
T0*
_output_shapes
:�
 Deterministic/sample/BroadcastToBroadcastTo Categorical/mode/ArgMax:output:0$Deterministic/sample/concat:output:0*
T0	*'
_output_shapes
:���������u
Deterministic/sample/Shape_1Shape)Deterministic/sample/BroadcastTo:output:0*
T0	*
_output_shapes
:t
*Deterministic/sample/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:v
,Deterministic/sample/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB: v
,Deterministic/sample/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:�
$Deterministic/sample/strided_slice_1StridedSlice%Deterministic/sample/Shape_1:output:03Deterministic/sample/strided_slice_1/stack:output:05Deterministic/sample/strided_slice_1/stack_1:output:05Deterministic/sample/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
:*
end_maskd
"Deterministic/sample/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B : �
Deterministic/sample/concat_1ConcatV2*Deterministic/sample/sample_shape:output:0-Deterministic/sample/strided_slice_1:output:0+Deterministic/sample/concat_1/axis:output:0*
N*
T0*
_output_shapes
:�
Deterministic/sample/ReshapeReshape)Deterministic/sample/BroadcastTo:output:0&Deterministic/sample/concat_1:output:0*
T0	*#
_output_shapes
:���������Y
clip_by_value/Minimum/yConst*
_output_shapes
: *
dtype0	*
value	B	 R�
clip_by_value/MinimumMinimum%Deterministic/sample/Reshape:output:0 clip_by_value/Minimum/y:output:0*
T0	*#
_output_shapes
:���������Q
clip_by_value/yConst*
_output_shapes
: *
dtype0	*
value	B	 R {
clip_by_valueMaximumclip_by_value/Minimum:z:0clip_by_value/y:output:0*
T0	*#
_output_shapes
:���������\
IdentityIdentityclip_by_value:z:0^NoOp*
T0	*#
_output_shapes
:����������
NoOpNoOp9^QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp9^QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp)^QNetwork/dense_14/BiasAdd/ReadVariableOp(^QNetwork/dense_14/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*_
_input_shapesN
L:���������:���������:���������:���������: : : : : : 2t
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp2t
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp2T
(QNetwork/dense_14/BiasAdd/ReadVariableOp(QNetwork/dense_14/BiasAdd/ReadVariableOp2R
'QNetwork/dense_14/MatMul/ReadVariableOp'QNetwork/dense_14/MatMul/ReadVariableOp:X T
#
_output_shapes
:���������
-
_user_specified_nametime_step/step_type:UQ
#
_output_shapes
:���������
*
_user_specified_nametime_step/reward:WS
#
_output_shapes
:���������
,
_user_specified_nametime_step/discount:^Z
'
_output_shapes
:���������
/
_user_specified_nametime_step/observation
�
�
-__inference_function_with_signature_126233504
	step_type

reward
discount
observation
unknown:	�
	unknown_0:	�
	unknown_1:
��
	unknown_2:	�
	unknown_3:	�
	unknown_4:
identity	��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCall	step_typerewarddiscountobservationunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
2
*
Tout
2	*
_collective_manager_ids
 *#
_output_shapes
:���������*(
_read_only_resource_inputs

	*0
config_proto 

CPU

GPU2*0J 8� *4
f/R-
+__inference_polymorphic_action_fn_126233489k
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0	*#
_output_shapes
:���������`
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*_
_input_shapesN
L:���������:���������:���������:���������: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:P L
#
_output_shapes
:���������
%
_user_specified_name0/step_type:MI
#
_output_shapes
:���������
"
_user_specified_name
0/reward:OK
#
_output_shapes
:���������
$
_user_specified_name
0/discount:VR
'
_output_shapes
:���������
'
_user_specified_name0/observation
�
e
__inference_<lambda>_126233323!
readvariableop_resource:	 
identity	��ReadVariableOp^
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
: *
dtype0	T
IdentityIdentityReadVariableOp:value:0^NoOp*
T0	*
_output_shapes
: W
NoOpNoOp^ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 2 
ReadVariableOpReadVariableOp
�
9
'__inference_get_initial_state_126233532

batch_size*(
_construction_contextkEagerRuntime*
_input_shapes
: :B >

_output_shapes
: 
$
_user_specified_name
batch_size
�
?
-__inference_function_with_signature_126233533

batch_size�
PartitionedCallPartitionedCall
batch_size*
Tin
2*

Tout
 *
_collective_manager_ids
 *
_output_shapes
 * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *0
f+R)
'__inference_get_initial_state_126233532*(
_construction_contextkEagerRuntime*
_input_shapes
: :B >

_output_shapes
: 
$
_user_specified_name
batch_size
�
m
-__inference_function_with_signature_126233545
unknown:	 
identity	��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallunknown*
Tin
2*
Tout
2	*
_collective_manager_ids
 *
_output_shapes
: *#
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *'
f"R 
__inference_<lambda>_126233323^
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0	*
_output_shapes
: `
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 22
StatefulPartitionedCallStatefulPartitionedCall
�L
�
+__inference_polymorphic_action_fn_126233489
	time_step
time_step_1
time_step_2
time_step_3S
@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource:	�P
Aqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource:	�T
@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource:
��P
Aqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource:	�C
0qnetwork_dense_14_matmul_readvariableop_resource:	�?
1qnetwork_dense_14_biasadd_readvariableop_resource:
identity	��8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp�8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp�(QNetwork/dense_14/BiasAdd/ReadVariableOp�'QNetwork/dense_14/MatMul/ReadVariableOpy
(QNetwork/EncodingNetwork/flatten_4/ConstConst*
_output_shapes
:*
dtype0*
valueB"����   �
*QNetwork/EncodingNetwork/flatten_4/ReshapeReshapetime_step_31QNetwork/EncodingNetwork/flatten_4/Const:output:0*
T0*'
_output_shapes
:����������
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
(QNetwork/EncodingNetwork/dense_12/MatMulMatMul3QNetwork/EncodingNetwork/flatten_4/Reshape:output:0?QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_12/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_12/MatMul:product:0@QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_12/TanhTanh2QNetwork/EncodingNetwork/dense_12/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource* 
_output_shapes
:
��*
dtype0�
(QNetwork/EncodingNetwork/dense_13/MatMulMatMul*QNetwork/EncodingNetwork/dense_12/Tanh:y:0?QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_13/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_13/MatMul:product:0@QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_13/TanhTanh2QNetwork/EncodingNetwork/dense_13/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
'QNetwork/dense_14/MatMul/ReadVariableOpReadVariableOp0qnetwork_dense_14_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
QNetwork/dense_14/MatMulMatMul*QNetwork/EncodingNetwork/dense_13/Tanh:y:0/QNetwork/dense_14/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:����������
(QNetwork/dense_14/BiasAdd/ReadVariableOpReadVariableOp1qnetwork_dense_14_biasadd_readvariableop_resource*
_output_shapes
:*
dtype0�
QNetwork/dense_14/BiasAddBiasAdd"QNetwork/dense_14/MatMul:product:00QNetwork/dense_14/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������l
!Categorical/mode/ArgMax/dimensionConst*
_output_shapes
: *
dtype0*
valueB :
����������
Categorical/mode/ArgMaxArgMax"QNetwork/dense_14/BiasAdd:output:0*Categorical/mode/ArgMax/dimension:output:0*
T0*#
_output_shapes
:���������T
Deterministic/atolConst*
_output_shapes
: *
dtype0	*
value	B	 R T
Deterministic/rtolConst*
_output_shapes
: *
dtype0	*
value	B	 R d
!Deterministic/sample/sample_shapeConst*
_output_shapes
: *
dtype0*
valueB j
Deterministic/sample/ShapeShape Categorical/mode/ArgMax:output:0*
T0	*
_output_shapes
:\
Deterministic/sample/ConstConst*
_output_shapes
: *
dtype0*
value	B : r
(Deterministic/sample/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: t
*Deterministic/sample/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:t
*Deterministic/sample/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:�
"Deterministic/sample/strided_sliceStridedSlice#Deterministic/sample/Shape:output:01Deterministic/sample/strided_slice/stack:output:03Deterministic/sample/strided_slice/stack_1:output:03Deterministic/sample/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_maskh
%Deterministic/sample/BroadcastArgs/s0Const*
_output_shapes
: *
dtype0*
valueB j
'Deterministic/sample/BroadcastArgs/s0_1Const*
_output_shapes
: *
dtype0*
valueB �
"Deterministic/sample/BroadcastArgsBroadcastArgs0Deterministic/sample/BroadcastArgs/s0_1:output:0+Deterministic/sample/strided_slice:output:0*
_output_shapes
:n
$Deterministic/sample/concat/values_0Const*
_output_shapes
:*
dtype0*
valueB:g
$Deterministic/sample/concat/values_2Const*
_output_shapes
: *
dtype0*
valueB b
 Deterministic/sample/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : �
Deterministic/sample/concatConcatV2-Deterministic/sample/concat/values_0:output:0'Deterministic/sample/BroadcastArgs:r0:0-Deterministic/sample/concat/values_2:output:0)Deterministic/sample/concat/axis:output:0*
N*
T0*
_output_shapes
:�
 Deterministic/sample/BroadcastToBroadcastTo Categorical/mode/ArgMax:output:0$Deterministic/sample/concat:output:0*
T0	*'
_output_shapes
:���������u
Deterministic/sample/Shape_1Shape)Deterministic/sample/BroadcastTo:output:0*
T0	*
_output_shapes
:t
*Deterministic/sample/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:v
,Deterministic/sample/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB: v
,Deterministic/sample/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:�
$Deterministic/sample/strided_slice_1StridedSlice%Deterministic/sample/Shape_1:output:03Deterministic/sample/strided_slice_1/stack:output:05Deterministic/sample/strided_slice_1/stack_1:output:05Deterministic/sample/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
:*
end_maskd
"Deterministic/sample/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B : �
Deterministic/sample/concat_1ConcatV2*Deterministic/sample/sample_shape:output:0-Deterministic/sample/strided_slice_1:output:0+Deterministic/sample/concat_1/axis:output:0*
N*
T0*
_output_shapes
:�
Deterministic/sample/ReshapeReshape)Deterministic/sample/BroadcastTo:output:0&Deterministic/sample/concat_1:output:0*
T0	*#
_output_shapes
:���������Y
clip_by_value/Minimum/yConst*
_output_shapes
: *
dtype0	*
value	B	 R�
clip_by_value/MinimumMinimum%Deterministic/sample/Reshape:output:0 clip_by_value/Minimum/y:output:0*
T0	*#
_output_shapes
:���������Q
clip_by_value/yConst*
_output_shapes
: *
dtype0	*
value	B	 R {
clip_by_valueMaximumclip_by_value/Minimum:z:0clip_by_value/y:output:0*
T0	*#
_output_shapes
:���������\
IdentityIdentityclip_by_value:z:0^NoOp*
T0	*#
_output_shapes
:����������
NoOpNoOp9^QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp9^QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp)^QNetwork/dense_14/BiasAdd/ReadVariableOp(^QNetwork/dense_14/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*_
_input_shapesN
L:���������:���������:���������:���������: : : : : : 2t
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp2t
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp2T
(QNetwork/dense_14/BiasAdd/ReadVariableOp(QNetwork/dense_14/BiasAdd/ReadVariableOp2R
'QNetwork/dense_14/MatMul/ReadVariableOp'QNetwork/dense_14/MatMul/ReadVariableOp:N J
#
_output_shapes
:���������
#
_user_specified_name	time_step:NJ
#
_output_shapes
:���������
#
_user_specified_name	time_step:NJ
#
_output_shapes
:���������
#
_user_specified_name	time_step:RN
'
_output_shapes
:���������
#
_user_specified_name	time_step
�
/
-__inference_function_with_signature_126233556�
PartitionedCallPartitionedCall*	
Tin
 *

Tout
 *
_collective_manager_ids
 *
_output_shapes
 * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *'
f"R 
__inference_<lambda>_126233326*(
_construction_contextkEagerRuntime*
_input_shapes 
�L
�
+__inference_polymorphic_action_fn_126233620
	step_type

reward
discount
observationS
@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource:	�P
Aqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource:	�T
@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource:
��P
Aqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource:	�C
0qnetwork_dense_14_matmul_readvariableop_resource:	�?
1qnetwork_dense_14_biasadd_readvariableop_resource:
identity	��8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp�8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp�(QNetwork/dense_14/BiasAdd/ReadVariableOp�'QNetwork/dense_14/MatMul/ReadVariableOpy
(QNetwork/EncodingNetwork/flatten_4/ConstConst*
_output_shapes
:*
dtype0*
valueB"����   �
*QNetwork/EncodingNetwork/flatten_4/ReshapeReshapeobservation1QNetwork/EncodingNetwork/flatten_4/Const:output:0*
T0*'
_output_shapes
:����������
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
(QNetwork/EncodingNetwork/dense_12/MatMulMatMul3QNetwork/EncodingNetwork/flatten_4/Reshape:output:0?QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_12/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_12/MatMul:product:0@QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_12/TanhTanh2QNetwork/EncodingNetwork/dense_12/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource* 
_output_shapes
:
��*
dtype0�
(QNetwork/EncodingNetwork/dense_13/MatMulMatMul*QNetwork/EncodingNetwork/dense_12/Tanh:y:0?QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_13/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_13/MatMul:product:0@QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_13/TanhTanh2QNetwork/EncodingNetwork/dense_13/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
'QNetwork/dense_14/MatMul/ReadVariableOpReadVariableOp0qnetwork_dense_14_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
QNetwork/dense_14/MatMulMatMul*QNetwork/EncodingNetwork/dense_13/Tanh:y:0/QNetwork/dense_14/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:����������
(QNetwork/dense_14/BiasAdd/ReadVariableOpReadVariableOp1qnetwork_dense_14_biasadd_readvariableop_resource*
_output_shapes
:*
dtype0�
QNetwork/dense_14/BiasAddBiasAdd"QNetwork/dense_14/MatMul:product:00QNetwork/dense_14/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������l
!Categorical/mode/ArgMax/dimensionConst*
_output_shapes
: *
dtype0*
valueB :
����������
Categorical/mode/ArgMaxArgMax"QNetwork/dense_14/BiasAdd:output:0*Categorical/mode/ArgMax/dimension:output:0*
T0*#
_output_shapes
:���������T
Deterministic/atolConst*
_output_shapes
: *
dtype0	*
value	B	 R T
Deterministic/rtolConst*
_output_shapes
: *
dtype0	*
value	B	 R d
!Deterministic/sample/sample_shapeConst*
_output_shapes
: *
dtype0*
valueB j
Deterministic/sample/ShapeShape Categorical/mode/ArgMax:output:0*
T0	*
_output_shapes
:\
Deterministic/sample/ConstConst*
_output_shapes
: *
dtype0*
value	B : r
(Deterministic/sample/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: t
*Deterministic/sample/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:t
*Deterministic/sample/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:�
"Deterministic/sample/strided_sliceStridedSlice#Deterministic/sample/Shape:output:01Deterministic/sample/strided_slice/stack:output:03Deterministic/sample/strided_slice/stack_1:output:03Deterministic/sample/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_maskh
%Deterministic/sample/BroadcastArgs/s0Const*
_output_shapes
: *
dtype0*
valueB j
'Deterministic/sample/BroadcastArgs/s0_1Const*
_output_shapes
: *
dtype0*
valueB �
"Deterministic/sample/BroadcastArgsBroadcastArgs0Deterministic/sample/BroadcastArgs/s0_1:output:0+Deterministic/sample/strided_slice:output:0*
_output_shapes
:n
$Deterministic/sample/concat/values_0Const*
_output_shapes
:*
dtype0*
valueB:g
$Deterministic/sample/concat/values_2Const*
_output_shapes
: *
dtype0*
valueB b
 Deterministic/sample/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : �
Deterministic/sample/concatConcatV2-Deterministic/sample/concat/values_0:output:0'Deterministic/sample/BroadcastArgs:r0:0-Deterministic/sample/concat/values_2:output:0)Deterministic/sample/concat/axis:output:0*
N*
T0*
_output_shapes
:�
 Deterministic/sample/BroadcastToBroadcastTo Categorical/mode/ArgMax:output:0$Deterministic/sample/concat:output:0*
T0	*'
_output_shapes
:���������u
Deterministic/sample/Shape_1Shape)Deterministic/sample/BroadcastTo:output:0*
T0	*
_output_shapes
:t
*Deterministic/sample/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB:v
,Deterministic/sample/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB: v
,Deterministic/sample/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:�
$Deterministic/sample/strided_slice_1StridedSlice%Deterministic/sample/Shape_1:output:03Deterministic/sample/strided_slice_1/stack:output:05Deterministic/sample/strided_slice_1/stack_1:output:05Deterministic/sample/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
:*
end_maskd
"Deterministic/sample/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B : �
Deterministic/sample/concat_1ConcatV2*Deterministic/sample/sample_shape:output:0-Deterministic/sample/strided_slice_1:output:0+Deterministic/sample/concat_1/axis:output:0*
N*
T0*
_output_shapes
:�
Deterministic/sample/ReshapeReshape)Deterministic/sample/BroadcastTo:output:0&Deterministic/sample/concat_1:output:0*
T0	*#
_output_shapes
:���������Y
clip_by_value/Minimum/yConst*
_output_shapes
: *
dtype0	*
value	B	 R�
clip_by_value/MinimumMinimum%Deterministic/sample/Reshape:output:0 clip_by_value/Minimum/y:output:0*
T0	*#
_output_shapes
:���������Q
clip_by_value/yConst*
_output_shapes
: *
dtype0	*
value	B	 R {
clip_by_valueMaximumclip_by_value/Minimum:z:0clip_by_value/y:output:0*
T0	*#
_output_shapes
:���������\
IdentityIdentityclip_by_value:z:0^NoOp*
T0	*#
_output_shapes
:����������
NoOpNoOp9^QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp9^QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp)^QNetwork/dense_14/BiasAdd/ReadVariableOp(^QNetwork/dense_14/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*_
_input_shapesN
L:���������:���������:���������:���������: : : : : : 2t
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp2t
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp2T
(QNetwork/dense_14/BiasAdd/ReadVariableOp(QNetwork/dense_14/BiasAdd/ReadVariableOp2R
'QNetwork/dense_14/MatMul/ReadVariableOp'QNetwork/dense_14/MatMul/ReadVariableOp:N J
#
_output_shapes
:���������
#
_user_specified_name	step_type:KG
#
_output_shapes
:���������
 
_user_specified_namereward:MI
#
_output_shapes
:���������
"
_user_specified_name
discount:TP
'
_output_shapes
:���������
%
_user_specified_nameobservation
�
)
'__inference_signature_wrapper_126233560�
PartitionedCallPartitionedCall*	
Tin
 *

Tout
 *
_collective_manager_ids
 *
_output_shapes
 * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *6
f1R/
-__inference_function_with_signature_126233556*(
_construction_contextkEagerRuntime*
_input_shapes 
�
9
'__inference_signature_wrapper_126233538

batch_size�
PartitionedCallPartitionedCall
batch_size*
Tin
2*

Tout
 *
_collective_manager_ids
 *
_output_shapes
 * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *6
f1R/
-__inference_function_with_signature_126233533*(
_construction_contextkEagerRuntime*
_input_shapes
: :B >

_output_shapes
: 
$
_user_specified_name
batch_size
�
g
'__inference_signature_wrapper_126233553
unknown:	 
identity	��StatefulPartitionedCall�
StatefulPartitionedCallStatefulPartitionedCallunknown*
Tin
2*
Tout
2	*
_collective_manager_ids
 *
_output_shapes
: *#
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8� *6
f1R/
-__inference_function_with_signature_126233545^
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0	*
_output_shapes
: `
NoOpNoOp^StatefulPartitionedCall*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*
_input_shapes
: 22
StatefulPartitionedCallStatefulPartitionedCall
_
 
__inference_<lambda>_126233326*(
_construction_contextkEagerRuntime*
_input_shapes 
�0
�
1__inference_polymorphic_distribution_fn_126233715
	step_type

reward
discount
observationS
@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource:	�P
Aqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource:	�T
@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource:
��P
Aqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource:	�C
0qnetwork_dense_14_matmul_readvariableop_resource:	�?
1qnetwork_dense_14_biasadd_readvariableop_resource:
identity	

identity_1	

identity_2	��8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp�8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp�7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp�(QNetwork/dense_14/BiasAdd/ReadVariableOp�'QNetwork/dense_14/MatMul/ReadVariableOpy
(QNetwork/EncodingNetwork/flatten_4/ConstConst*
_output_shapes
:*
dtype0*
valueB"����   �
*QNetwork/EncodingNetwork/flatten_4/ReshapeReshapeobservation1QNetwork/EncodingNetwork/flatten_4/Const:output:0*
T0*'
_output_shapes
:����������
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_12_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
(QNetwork/EncodingNetwork/dense_12/MatMulMatMul3QNetwork/EncodingNetwork/flatten_4/Reshape:output:0?QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_12_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_12/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_12/MatMul:product:0@QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_12/TanhTanh2QNetwork/EncodingNetwork/dense_12/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOpReadVariableOp@qnetwork_encodingnetwork_dense_13_matmul_readvariableop_resource* 
_output_shapes
:
��*
dtype0�
(QNetwork/EncodingNetwork/dense_13/MatMulMatMul*QNetwork/EncodingNetwork/dense_12/Tanh:y:0?QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOpReadVariableOpAqnetwork_encodingnetwork_dense_13_biasadd_readvariableop_resource*
_output_shapes	
:�*
dtype0�
)QNetwork/EncodingNetwork/dense_13/BiasAddBiasAdd2QNetwork/EncodingNetwork/dense_13/MatMul:product:0@QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:�����������
&QNetwork/EncodingNetwork/dense_13/TanhTanh2QNetwork/EncodingNetwork/dense_13/BiasAdd:output:0*
T0*(
_output_shapes
:�����������
'QNetwork/dense_14/MatMul/ReadVariableOpReadVariableOp0qnetwork_dense_14_matmul_readvariableop_resource*
_output_shapes
:	�*
dtype0�
QNetwork/dense_14/MatMulMatMul*QNetwork/EncodingNetwork/dense_13/Tanh:y:0/QNetwork/dense_14/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:����������
(QNetwork/dense_14/BiasAdd/ReadVariableOpReadVariableOp1qnetwork_dense_14_biasadd_readvariableop_resource*
_output_shapes
:*
dtype0�
QNetwork/dense_14/BiasAddBiasAdd"QNetwork/dense_14/MatMul:product:00QNetwork/dense_14/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:���������l
!Categorical/mode/ArgMax/dimensionConst*
_output_shapes
: *
dtype0*
valueB :
����������
Categorical/mode/ArgMaxArgMax"QNetwork/dense_14/BiasAdd:output:0*Categorical/mode/ArgMax/dimension:output:0*
T0*#
_output_shapes
:���������T
Deterministic/atolConst*
_output_shapes
: *
dtype0	*
value	B	 R T
Deterministic/rtolConst*
_output_shapes
: *
dtype0	*
value	B	 R Y
IdentityIdentityDeterministic/atol:output:0^NoOp*
T0	*
_output_shapes
: m

Identity_1Identity Categorical/mode/ArgMax:output:0^NoOp*
T0	*#
_output_shapes
:���������[

Identity_2IdentityDeterministic/rtol:output:0^NoOp*
T0	*
_output_shapes
: �
NoOpNoOp9^QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp9^QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8^QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp)^QNetwork/dense_14/BiasAdd/ReadVariableOp(^QNetwork/dense_14/MatMul/ReadVariableOp*"
_acd_function_control_output(*
_output_shapes
 "
identityIdentity:output:0"!

identity_1Identity_1:output:0"!

identity_2Identity_2:output:0*(
_construction_contextkEagerRuntime*_
_input_shapesN
L:���������:���������:���������:���������: : : : : : 2t
8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_12/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_12/MatMul/ReadVariableOp2t
8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp8QNetwork/EncodingNetwork/dense_13/BiasAdd/ReadVariableOp2r
7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp7QNetwork/EncodingNetwork/dense_13/MatMul/ReadVariableOp2T
(QNetwork/dense_14/BiasAdd/ReadVariableOp(QNetwork/dense_14/BiasAdd/ReadVariableOp2R
'QNetwork/dense_14/MatMul/ReadVariableOp'QNetwork/dense_14/MatMul/ReadVariableOp:N J
#
_output_shapes
:���������
#
_user_specified_name	step_type:KG
#
_output_shapes
:���������
 
_user_specified_namereward:MI
#
_output_shapes
:���������
"
_user_specified_name
discount:TP
'
_output_shapes
:���������
%
_user_specified_nameobservation
�
�
"__inference__traced_save_126233767
file_prefix'
#savev2_variable_read_readvariableop	G
Csavev2_qnetwork_encodingnetwork_dense_12_kernel_read_readvariableopE
Asavev2_qnetwork_encodingnetwork_dense_12_bias_read_readvariableopG
Csavev2_qnetwork_encodingnetwork_dense_13_kernel_read_readvariableopE
Asavev2_qnetwork_encodingnetwork_dense_13_bias_read_readvariableop7
3savev2_qnetwork_dense_14_kernel_read_readvariableop5
1savev2_qnetwork_dense_14_bias_read_readvariableop
savev2_const

identity_1��MergeV2Checkpointsw
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*Z
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.parta
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part�
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: f

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: L

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :f
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : �
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: �
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*�
value�B�B%train_step/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/0/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/1/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/2/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/3/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/4/.ATTRIBUTES/VARIABLE_VALUEB,model_variables/5/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH}
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*#
valueBB B B B B B B B �
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0#savev2_variable_read_readvariableopCsavev2_qnetwork_encodingnetwork_dense_12_kernel_read_readvariableopAsavev2_qnetwork_encodingnetwork_dense_12_bias_read_readvariableopCsavev2_qnetwork_encodingnetwork_dense_13_kernel_read_readvariableopAsavev2_qnetwork_encodingnetwork_dense_13_bias_read_readvariableop3savev2_qnetwork_dense_14_kernel_read_readvariableop1savev2_qnetwork_dense_14_bias_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 *
dtypes

2	�
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:�
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 f
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: Q

Identity_1IdentityIdentity:output:0^NoOp*
T0*
_output_shapes
: [
NoOpNoOp^MergeV2Checkpoints*"
_acd_function_control_output(*
_output_shapes
 "!

identity_1Identity_1:output:0*O
_input_shapes>
<: : :	�:�:
��:�:	�:: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix:

_output_shapes
: :%!

_output_shapes
:	�:!

_output_shapes	
:�:&"
 
_output_shapes
:
��:!

_output_shapes	
:�:%!

_output_shapes
:	�: 

_output_shapes
::

_output_shapes
: "�L
saver_filename:0StatefulPartitionedCall_2:0StatefulPartitionedCall_38"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*�
action�
4

0/discount&
action_0_discount:0���������
>
0/observation-
action_0_observation:0���������
0
0/reward$
action_0_reward:0���������
6
0/step_type'
action_0_step_type:0���������6
action,
StatefulPartitionedCall:0	���������tensorflow/serving/predict*e
get_initial_stateP
2

batch_size$
get_initial_state_batch_size:0 tensorflow/serving/predict*,
get_metadatatensorflow/serving/predict*Z
get_train_stepH*
int64!
StatefulPartitionedCall_1:0	 tensorflow/serving/predict:�h
�

train_step
metadata
model_variables
_all_assets

action
distribution
get_initial_state
get_metadata
	get_train_step


signatures"
_generic_user_object
:	 (2Variable
 "
trackable_dict_wrapper
K
0
1
2
3
4
5"
trackable_tuple_wrapper
5
_wrapped_policy"
trackable_dict_wrapper
�
trace_0
trace_12�
+__inference_polymorphic_action_fn_126233620
+__inference_polymorphic_action_fn_126233680�
���
FullArgSpec(
args �
j	time_step
jpolicy_state
varargs
 
varkw
 
defaults�
� 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 ztrace_0ztrace_1
�
trace_02�
1__inference_polymorphic_distribution_fn_126233715�
���
FullArgSpec(
args �
j	time_step
jpolicy_state
varargs
 
varkw
 
defaults�
� 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 ztrace_0
�
trace_02�
'__inference_get_initial_state_126233718�
���
FullArgSpec!
args�
jself
j
batch_size
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 ztrace_0
�B�
__inference_<lambda>_126233326"�
���
FullArgSpec
args� 
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
__inference_<lambda>_126233323"�
���
FullArgSpec
args� 
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
`

action
get_initial_state
get_train_step
get_metadata"
signature_map
;:9	�2(QNetwork/EncodingNetwork/dense_12/kernel
5:3�2&QNetwork/EncodingNetwork/dense_12/bias
<::
��2(QNetwork/EncodingNetwork/dense_13/kernel
5:3�2&QNetwork/EncodingNetwork/dense_13/bias
+:)	�2QNetwork/dense_14/kernel
$:"2QNetwork/dense_14/bias
.

_q_network"
_generic_user_object
�B�
+__inference_polymorphic_action_fn_126233620	step_typerewarddiscountobservation"�
���
FullArgSpec(
args �
j	time_step
jpolicy_state
varargs
 
varkw
 
defaults�
� 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
+__inference_polymorphic_action_fn_126233680time_step/step_typetime_step/rewardtime_step/discounttime_step/observation"�
���
FullArgSpec(
args �
j	time_step
jpolicy_state
varargs
 
varkw
 
defaults�
� 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
1__inference_polymorphic_distribution_fn_126233715	step_typerewarddiscountobservation"�
���
FullArgSpec(
args �
j	time_step
jpolicy_state
varargs
 
varkw
 
defaults�
� 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
'__inference_get_initial_state_126233718
batch_size"�
���
FullArgSpec!
args�
jself
j
batch_size
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
'__inference_signature_wrapper_126233526
0/discount0/observation0/reward0/step_type"�
���
FullArgSpec
args� 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
'__inference_signature_wrapper_126233538
batch_size"�
���
FullArgSpec
args� 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
'__inference_signature_wrapper_126233553"�
���
FullArgSpec
args� 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�B�
'__inference_signature_wrapper_126233560"�
���
FullArgSpec
args� 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�
	variables
trainable_variables
regularization_losses
	keras_api
__call__
* &call_and_return_all_conditional_losses
!_encoder
"_q_value_layer"
_tf_keras_layer
J
0
1
2
3
4
5"
trackable_list_wrapper
J
0
1
2
3
4
5"
trackable_list_wrapper
 "
trackable_list_wrapper
�
#non_trainable_variables

$layers
%metrics
&layer_regularization_losses
'layer_metrics
	variables
trainable_variables
regularization_losses
__call__
* &call_and_return_all_conditional_losses
& "call_and_return_conditional_losses"
_generic_user_object
�2��
���
FullArgSpecL
argsD�A
jself
jobservation
j	step_type
jnetwork_state

jtraining
varargs
 
varkw
 
defaults�

 
� 
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
�2��
���
FullArgSpecL
argsD�A
jself
jobservation
j	step_type
jnetwork_state

jtraining
varargs
 
varkw
 
defaults�

 
� 
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
�
(	variables
)trainable_variables
*regularization_losses
+	keras_api
,__call__
*-&call_and_return_all_conditional_losses
._postprocessing_layers"
_tf_keras_layer
�
/	variables
0trainable_variables
1regularization_losses
2	keras_api
3__call__
*4&call_and_return_all_conditional_losses

kernel
bias"
_tf_keras_layer
 "
trackable_list_wrapper
.
!0
"1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
<
0
1
2
3"
trackable_list_wrapper
<
0
1
2
3"
trackable_list_wrapper
 "
trackable_list_wrapper
�
5non_trainable_variables

6layers
7metrics
8layer_regularization_losses
9layer_metrics
(	variables
)trainable_variables
*regularization_losses
,__call__
*-&call_and_return_all_conditional_losses
&-"call_and_return_conditional_losses"
_generic_user_object
�2��
���
FullArgSpecL
argsD�A
jself
jobservation
j	step_type
jnetwork_state

jtraining
varargs
 
varkw
 
defaults�

 
� 
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
�2��
���
FullArgSpecL
argsD�A
jself
jobservation
j	step_type
jnetwork_state

jtraining
varargs
 
varkw
 
defaults�

 
� 
p 

kwonlyargs� 
kwonlydefaults� 
annotations� *
 
5
:0
;1
<2"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
=non_trainable_variables

>layers
?metrics
@layer_regularization_losses
Alayer_metrics
/	variables
0trainable_variables
1regularization_losses
3__call__
*4&call_and_return_all_conditional_losses
&4"call_and_return_conditional_losses"
_generic_user_object
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
 "
trackable_list_wrapper
5
:0
;1
<2"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
�
B	variables
Ctrainable_variables
Dregularization_losses
E	keras_api
F__call__
*G&call_and_return_all_conditional_losses"
_tf_keras_layer
�
H	variables
Itrainable_variables
Jregularization_losses
K	keras_api
L__call__
*M&call_and_return_all_conditional_losses

kernel
bias"
_tf_keras_layer
�
N	variables
Otrainable_variables
Pregularization_losses
Q	keras_api
R__call__
*S&call_and_return_all_conditional_losses

kernel
bias"
_tf_keras_layer
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
�
Tnon_trainable_variables

Ulayers
Vmetrics
Wlayer_regularization_losses
Xlayer_metrics
B	variables
Ctrainable_variables
Dregularization_losses
F__call__
*G&call_and_return_all_conditional_losses
&G"call_and_return_conditional_losses"
_generic_user_object
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
Ynon_trainable_variables

Zlayers
[metrics
\layer_regularization_losses
]layer_metrics
H	variables
Itrainable_variables
Jregularization_losses
L__call__
*M&call_and_return_all_conditional_losses
&M"call_and_return_conditional_losses"
_generic_user_object
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
�
^non_trainable_variables

_layers
`metrics
alayer_regularization_losses
blayer_metrics
N	variables
Otrainable_variables
Pregularization_losses
R__call__
*S&call_and_return_all_conditional_losses
&S"call_and_return_conditional_losses"
_generic_user_object
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
�2��
���
FullArgSpec
args�
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs� 
kwonlydefaults
 
annotations� *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper=
__inference_<lambda>_126233323�

� 
� "� 	6
__inference_<lambda>_126233326�

� 
� "� T
'__inference_get_initial_state_126233718)"�
�
�

batch_size 
� "� �
+__inference_polymorphic_action_fn_126233620����
���
���
TimeStep,
	step_type�
	step_type���������&
reward�
reward���������*
discount�
discount���������4
observation%�"
observation���������
� 
� "R�O

PolicyStep&
action�
action���������	
state� 
info� �
+__inference_polymorphic_action_fn_126233680����
���
���
TimeStep6
	step_type)�&
time_step/step_type���������0
reward&�#
time_step/reward���������4
discount(�%
time_step/discount���������>
observation/�,
time_step/observation���������
� 
� "R�O

PolicyStep&
action�
action���������	
state� 
info� �
1__inference_polymorphic_distribution_fn_126233715����
���
���
TimeStep,
	step_type�
	step_type���������&
reward�
reward���������*
discount�
discount���������4
observation%�"
observation���������
� 
� "���

PolicyStep�
action������
`
B�?

atol� 	

loc����������	

rtol� 	
J�G

allow_nan_statsp

namejDeterministic_1

validate_argsp 
�
j
parameters
� 
�
jname+tfp.distributions.Deterministic_ACTTypeSpec 
state� 
info� �
'__inference_signature_wrapper_126233526����
� 
���
.

0/discount �

0/discount���������
8
0/observation'�$
0/observation���������
*
0/reward�
0/reward���������
0
0/step_type!�
0/step_type���������"+�(
&
action�
action���������	b
'__inference_signature_wrapper_12623353870�-
� 
&�#
!

batch_size�

batch_size "� [
'__inference_signature_wrapper_1262335530�

� 
� "�

int64�
int64 	?
'__inference_signature_wrapper_126233560�

� 
� "� 